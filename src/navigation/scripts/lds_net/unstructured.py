"""Bounded U0/U1/LDS-Net progressive scalar pruning; no shape or graph surgery."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import os
import time
from pathlib import Path
import torch
from torch.utils.data import DataLoader
from ultralytics.models.yolo.detect import DetectionTrainer
from .data import ROOT, COCO, seed_worker, seg_loader, write_json
from .training import RoadTrainer, load_model
from .quant_training import CoreBranch, PairedRoadSampler
from .deploy_quant import QConv, TensorQuant
from .depth_compression import (CompressionTrainingModel, CompressionTrainer, kd_gain,
                                compression_acceptance)
from .aggressive_data import DeterministicRoadDataset, reset_detection_epoch
from .aggressive_training import pad_detection_batch
from .ablation_metrics import road_detailed
from .quant_evaluation import ATTR_ROOT, validation_ids
from .sparse_weights import graph_package

TARGETS = {
    'U0': dict(zip(('model.5.conv','model.7.conv','model.8.expand.conv','model.11.conv'), (0.,0.,0.,0.))),
    'U1': dict(zip(('model.5.conv','model.7.conv','model.8.expand.conv','model.11.conv'), (.4,.5,.5,.3))),
    'LDS-Net': dict(zip(('model.5.conv','model.7.conv','model.8.expand.conv','model.11.conv'), (.5,.65,.65,.4))),
}
UPDATES_PER_EPOCH = 107
BASELINE_BYTES = 240640
RUN_GROUPS = ('U1','LDS-Net')
EPOCHS = 8
WARMUP_EPOCHS = 1
PRUNE_END_EPOCH = 4
VALIDATE_EPOCHS = (4,6,8)


class StopRequested(BaseException):
    pass


def masked_ops(model):
    return [(name, m) for name,m in model.named_modules() if isinstance(m,QConv) and hasattr(m,'weight_mask')]


def sparse_policy(model, update_weights=False):
    for m in model.modules():
        if isinstance(m,TensorQuant):
            m.enabled, m.observe = True, False
        elif isinstance(m,QConv):
            m.enabled, m.update_weights = True, update_weights


@torch.no_grad()
def enforce_masks(model, optimizer=None, ema=None):
    for name, op in masked_ops(model):
        mask, p = op.weight_mask, op.conv.weight
        p.mul_(mask)
        if p.grad is not None:
            p.grad.mul_(mask)
        if optimizer is not None:
            for value in optimizer.state.get(p,{}).values():
                if isinstance(value,torch.Tensor) and value.shape == p.shape:
                    value.mul_(mask)
        if ema is not None:
            other = ema.get_submodule(name)
            other.weight_mask.copy_(mask)
            other.conv.weight.mul_(mask)


def pruning_progress(updates):
    duration=(PRUNE_END_EPOCH-WARMUP_EPOCHS)*UPDATES_PER_EPOCH
    t = min(max(int(updates)-WARMUP_EPOCHS*UPDATES_PER_EPOCH,0),duration)
    return 1-(1-t/duration)**3


@torch.no_grad()
def refresh_masks(model, force=False):
    updates = int(model.sparse_updates)
    t = updates-WARMUP_EPOCHS*UPDATES_PER_EPOCH
    duration=(PRUNE_END_EPOCH-WARMUP_EPOCHS)*UPDATES_PER_EPOCH
    if not force and not (0 < t <= duration and (t%16 == 0 or t==duration)):
        return False
    fraction = pruning_progress(updates)
    targets = TARGETS[model.sparse_group]
    for row in model.core.nodes:
        if row.get('source') not in targets:
            continue
        op = model.core.ops[row['id']]
        w, mask = op.conv.weight.flatten(1), op.weight_mask.flatten(1)
        target = math.floor(w.shape[1]*targets[row['source']]*fraction)
        for channel in range(len(w)):
            need = target-int((~mask[channel]).sum())
            if need < 0:
                raise AssertionError('Pruning cannot regrow connections')
            if need:
                # Already pruned positions are ineligible, including ties at 0.
                score = w[channel].abs().masked_fill(~mask[channel],float('inf'))
                order = torch.argsort(score,stable=True)
                mask[channel,order[:need]] = False
    return True


def mask_report(model):
    rows = []; zeros = 0; n = 0
    for row in model.core.nodes:
        if row['op'] != 'conv':
            continue
        op = model.core.ops[row['id']]
        n += op.conv.weight.numel()
        if not hasattr(op,'weight_mask'):
            continue
        count = int((~op.weight_mask).sum()); zeros += count
        target = math.floor(op.conv.weight[0].numel()*TARGETS[model.sparse_group][row['source']])
        rows.append({'source':row['source'],'weights':op.conv.weight.numel(),'masked':count,
                     'masked_fraction':count/op.conv.weight.numel(),
                     'per_channel_masked':(~op.weight_mask.flatten(1)).sum(1).cpu().tolist(),
                     'final_target_per_channel':target})
    return {'masked':zeros,'weights':n,'mask_sparsity':zeros/max(n,1),'layers':rows,
            'target_reached':all(all(c==r['final_target_per_channel'] for c in r['per_channel_masked']) for r in rows)}


class SparseTrainingModel(CompressionTrainingModel):
    @classmethod
    def from_training(cls, original, group):
        if group not in TARGETS:
            raise ValueError(group)
        model = copy.deepcopy(original).cpu().float()
        model.__class__ = cls
        model.sparse_group = group
        model.register_buffer('sparse_updates',torch.zeros((),dtype=torch.long))
        for row in model.core.nodes:
            if row.get('source') in TARGETS[group]:
                op = model.core.ops[row['id']]
                op.register_buffer('weight_mask',torch.ones_like(op.conv.weight,dtype=torch.bool))
        for m in model.modules():
            if isinstance(m,CoreBranch):
                object.__setattr__(m,'core',model.core)
        model.criterion = None
        model.kd_gain = 0.
        model.requires_grad_(True)
        model.attach_teacher(None)
        object.__setattr__(model,'_det_features',None)
        sparse_policy(model)
        return model


def gate_for(report, baseline, storage, group):
    result = compression_acceptance(report,baseline)
    result['accuracy_eligible'] = result['eligible']
    result['weight_file_bytes'] = storage['file_bytes']
    result['weight_reduction'] = 1-storage['file_bytes']/BASELINE_BYTES
    result['storage_eligible'] = storage['file_bytes'] <= BASELINE_BYTES*.9
    result['eligible'] = result['accuracy_eligible'] and result['storage_eligible'] and group!='U0'
    result['control'] = group=='U0'
    return result


def candidate_rank(gate):
    v = gate['values']
    if gate['control']:
        return (float(gate['accuracy_eligible']),-gate['total_deficit'],v['map'],v['foreground_miou'])
    return (float(gate['eligible']),-gate['total_deficit'],
            -max(0.,.1-gate['weight_reduction']),-gate['weight_file_bytes'],
            v['map'],v['foreground_miou'],-v['alternative_as_direct'])


class SparseTrainer(CompressionTrainer):
    def __init__(self, **kwargs):
        self.cancel_pending = False
        super().__init__(mode='sparse',**kwargs)

    def audit_start(self, trainer):
        super().audit_start(trainer)
        assert self.accumulate == 4
        assert int(self.model.sparse_updates)==self.start_epoch*UPDATES_PER_EPOCH
        # The engine initializes last_opt_step=-1 even when ni resumes far from
        # zero. Avoid its premature first-batch update on an epoch resume.
        if self.start_epoch:
            self.accumulate = self.start_epoch*428+4
        self.initial_activation_scales = {n:m.scale.clone() for n,m in self.model.named_modules() if isinstance(m,TensorQuant)}
        self.sync_ema()

    def epoch_start(self, trainer):
        RoadTrainer.epoch_start(self,trainer)
        reset_detection_epoch(self.train_loader,self.epoch)
        sparse_policy(self.model, self.epoch<PRUNE_END_EPOCH)
        self.sync_ema()
        if self.road_train_loader is not None:
            self.road_train_loader.batch_sampler.set_epoch(self.epoch)
            self.road_iterator = iter(self.road_train_loader)

    def sync_ema(self):
        super().sync_ema()
        for name,op in masked_ops(self.model):
            self.ema.ema.get_submodule(name).weight_mask.copy_(op.weight_mask)
        self.ema.ema.sparse_updates.copy_(self.model.sparse_updates)
        sparse_policy(self.ema.ema,False)

    def before_batch(self, _):
        assert self.batch_size==16 and len(self.train_loader)==428
        self.model.kd_gain = min(1.,(self.epoch+self.epoch_batches/428)/WARMUP_EPOCHS)
        self.cancel_pending |= (self.experiment.parent/'STOP_REQUESTED.json').exists()

    def optimizer_step(self):
        enforce_masks(self.model,self.optimizer)
        super().optimizer_step()
        self.accumulate = 4
        self.model.sparse_updates.add_(1)
        refresh_masks(self.model)
        enforce_masks(self.model,self.optimizer,self.ema.ema)
        if self.epoch<PRUNE_END_EPOCH:
            for op in self.model.modules():
                if isinstance(op,QConv):
                    op.initialize_weight_scale()
        self.sync_ema()
        if self.cancel_pending or (self.experiment.parent/'STOP_REQUESTED.json').exists():
            self.save_checkpoint(self.wdir/'cancelled_step.pt',partial=True)
            raise StopRequested('Stopped at a complete optimizer step; remaining groups cancelled')

    def preprocess_batch(self, batch):
        batch = pad_detection_batch(DetectionTrainer.preprocess_batch(self,batch),16)
        if self.road_train_loader is None:
            self.road_train_loader = DataLoader(DeterministicRoadDataset(),
                batch_sampler=PairedRoadSampler(8882,epoch=self.epoch),num_workers=2,
                worker_init_fn=seed_worker,pin_memory=True)
            self.road_iterator = iter(self.road_train_loader)
        road = next(self.road_iterator)
        batch['seg_img'] = road['img'].to(self.device,non_blocking=True).float()/255
        batch['seg_mask'] = road['mask'].to(self.device,non_blocking=True)
        if self.epoch==0 and self.epoch_batches<2:
            path = self.save_dir/'paired_batch_hashes.json'
            records = json.loads(path.read_text()) if path.exists() else {}
            record = {k:hashlib.sha256(batch[k].detach().cpu().numpy().tobytes()).hexdigest()
                      for k in ('img','seg_img','seg_mask')}
            record['detection_files'] = [Path(p).name for p in batch['im_file']]
            record['road_ids'] = road['id']
            records[str(self.epoch_batches)] = record
            write_json(path,records)
            control = self.experiment.parent/RUN_GROUPS[0]/'training/qat/paired_batch_hashes.json'
            if self.group!=RUN_GROUPS[0]:
                assert json.loads(control.read_text())[str(self.epoch_batches)]==record
        return batch

    def publish_progress(self):
        status = self.experiment/'status.json'
        if not status.exists():
            return
        row = json.loads(status.read_text(encoding='utf-8'))
        revision = row.get('time')
        if revision == getattr(self,'_sparse_progress_revision',None):
            return
        row.update(group=self.group,stage='W4A6_sparse_QAT',
            optimizer_updates=int(self.model.sparse_updates),
            pruning_phase='warmup' if self.epoch<WARMUP_EPOCHS else ('progressive' if self.epoch<PRUNE_END_EPOCH else 'fixed_mask_recovery'),
            activation_scales_frozen=True,weight_scales_frozen=self.epoch>=PRUNE_END_EPOCH,
            deployment_unchanged=True)
        try:
            write_json(self.experiment.parent/'status.json',row)
            self._sparse_progress_revision=revision
        except PermissionError as error:
            print('PROGRESS_WRITE_DEFERRED',error,flush=True)

    def validate(self):
        assert int(self.model.sparse_updates)==(self.epoch+1)*UPDATES_PER_EPOCH
        for name,m in self.model.named_modules():
            if isinstance(m,TensorQuant):
                assert torch.equal(m.scale,self.initial_activation_scales[name]),name
        if self.epoch+1 not in VALIDATE_EPOCHS:
            if self.best_fitness is None:
                self.best_fitness = -1000.
            print(f'SPARSE_VALIDATION_DEFERRED {self.group} epoch={self.epoch+1}',flush=True)
            return self.validation_metrics(self.metrics,deferred=True),-float(self.epoch+1)
        model = self.ema.ema
        sparse_policy(model,False)
        self.validator.size_counts = {k:{'targets':0,'matched':0} for k in ('short_lt8','short_8_to16','short_ge16')}
        metrics = self.validator(self)
        metrics.pop('fitness',None)
        if self.road_val_loader is None:
            self.road_val_loader = seg_loader('val',batch=16,workers=0,ids=validation_ids())
        attrs = json.loads((ATTR_ROOT/'attributes.json').read_text(encoding='utf-8'))
        road = road_detailed(model,attrs,device=str(self.device),loader=self.road_val_loader)
        report = {'detection':metrics,'road':road,'small_targets':self.validator.size_counts,
            'detection_per_class':{str(k):float(v) for k,v in zip(self.validator.metrics.box.ap_class_index,self.validator.metrics.box.ap)}}
        _,storage = graph_package(model.core)
        gate = gate_for(report,self.baseline,storage,self.group)
        masks = mask_report(model)
        selectable = self.epoch>=PRUNE_END_EPOCH and masks['target_reached']
        rank = candidate_rank(gate)
        improved = selectable and (self.best_rank is None or rank>self.best_rank)
        if improved:
            self.best_rank,self.best_epoch = rank,self.epoch
            self.best_fitness = float(self.epoch+1)
        if self.best_fitness is None:
            self.best_fitness = -1000.
        fitness = self.best_fitness if improved else -float(self.epoch+1)
        self.latest_road = road
        metrics.update({'road/foreground_miou':road['foreground_miou'],
            'road/iou_direct':road['iou'][1],'road/iou_alternative':road['iou'][2],
            'road/boundary_f1_2px_320':road['boundary']['2']['f1'],
            'sparse/weight_file_bytes':storage['file_bytes'],'sparse/mask_fraction':masks['mask_sparsity'],
            'sparse/integer_zero_fraction':storage['integer_zero_fraction'],'selection/eligible':float(gate['eligible'])})
        write_json(self.save_dir/'epoch_metrics'/f'{self.epoch+1:03d}.json',{
            'group':self.group,'epoch':self.epoch+1,'selectable':selectable,'new_best':improved,
            'gate':gate,'report':report,'masks':masks,'storage':storage,
            'optimizer_updates':int(self.model.sparse_updates)})
        print(f'SPARSE_EPOCH {self.group} {self.epoch+1}/{EPOCHS} mAP={gate["values"]["map"]:.5f} '
              f'road={road["foreground_miou"]:.5f} bytes={storage["file_bytes"]} eligible={gate["eligible"]}',flush=True)
        return self.validation_metrics(metrics,deferred=False),fitness

    def validation_metrics(self, metrics, deferred):
        # Keep CSV columns/order stable across skipped and actual validation.
        keys=list(self.validator.metrics.keys)+list(self.label_loss_items(prefix='val'))+[
            'road/foreground_miou','road/iou_direct','road/iou_alternative',
            'road/boundary_f1_2px_320','sparse/weight_file_bytes','sparse/mask_fraction',
            'sparse/integer_zero_fraction','selection/eligible','validation/deferred']
        return {k:float(deferred) if k=='validation/deferred' else float(metrics.get(k,0.)) for k in keys}

    def save_checkpoint(self,path,partial=False):
        live = copy.deepcopy(self.model).cpu().float(); live.criterion=None
        ema = copy.deepcopy(self.ema.ema).cpu().float(); ema.criterion=None
        checkpoint = {'epoch':self.epoch,'best_fitness':self.best_fitness,'model':live,'ema':ema,
            'updates':self.ema.updates,'optimizer':self.optimizer.state_dict(),
            'scaler':self.scaler.state_dict(),'train_args':vars(self.args),'train_metrics':self.metrics,
            'train_results':self.read_results_csv(),'selection_rank':self.best_rank,'selected_epoch':self.best_epoch,
            'phase':'unstructured_W4A6','modelopt':None,'version':'8.4.163','precision':'FP32',
            'scheduler_state':self.scheduler.state_dict(),'partial_epoch':partial,
            'next_batch':self.epoch_batches+1 if partial else 0,
            'sparse_optimizer_updates':int(self.model.sparse_updates),'sparse_targets':TARGETS[self.group]}
        path = Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        temp = path.with_suffix('.pt.tmp')
        torch.save(checkpoint,temp)
        for attempt in range(9):
            try:
                temp.replace(path);break
            except PermissionError:
                if attempt==8:raise
                time.sleep(min(.025*2**attempt,.25))

    def save_model(self):
        self.save_checkpoint(self.last)
        if self.fitness==self.best_fitness:
            self.save_checkpoint(self.best)
        return True


def train_sparse(model,teacher,root,group,baseline,manifest):
    out = Path(root)/group
    stage = out/'training/qat'
    last,complete = stage/'weights/last.pt',stage/'completion.json'
    if complete.exists():
        assert json.loads(complete.read_text())['epochs_completed']==EPOCHS
        return load_model(stage/'weights/best.pt')
    model.requires_grad_(True)
    overrides = {'model':'yolo26n.yaml','data':str(COCO),'task':'detect','pretrained':False,
        'epochs':EPOCHS,'patience':0,'imgsz':320,'batch':16,'nbs':64,'workers':4,'device':0,
        'optimizer':'AdamW','lr0':1e-5,'lrf':.1,'weight_decay':.0005,'cos_lr':True,'warmup_epochs':0.,
        'amp':False,'seed':20261017,'deterministic':True,'rect':False,'multi_scale':0.,
        'mosaic':0.,'close_mosaic':0,'mixup':0.,'cutmix':0.,'scale':.3,'translate':.1,'fliplr':.5,
        'save':True,'save_period':0,'plots':False,'val':True,'max_det':300,'nms':False,
        'project':str(out/'training'),'name':'qat','exist_ok':True,'save_dir':str(stage),'verbose':False}
    checkpoint=None
    if last.exists():
        checkpoint=torch.load(last,map_location='cpu',weights_only=False)
        assert not checkpoint['partial_epoch'],'Explicit partial-step replay is not an epoch resume'
        if checkpoint['epoch']+1>=EPOCHS:
            return load_model(stage/'weights/best.pt')
        overrides.update(model=str(last),resume=str(last))
    trainer=SparseTrainer(group=group,teacher=teacher,baseline=baseline,manifest=manifest,
        experiment=out,stage_name='W4A6_sparse_QAT',joint=True,overrides=overrides)
    trainer.model=checkpoint['model'] if checkpoint else model
    if checkpoint:
        trainer.model.requires_grad_(True)
        original_resume=trainer.resume_training
        trainer.resume_training=lambda _:original_resume(checkpoint)
    try:
        trainer.train()
    finally:
        trainer.model.attach_teacher(None)
        for loader in (trainer.road_train_loader,trainer.road_val_loader):
            iterator=getattr(loader,'_iterator',None)
            if iterator is not None:iterator._shutdown_workers()
        iterator=getattr(trainer,'road_iterator',None)
        if iterator is not None and hasattr(iterator,'_shutdown_workers'):iterator._shutdown_workers()
        if hasattr(trainer.train_loader,'close'):trainer.train_loader.close()
    return load_model(trainer.best)
