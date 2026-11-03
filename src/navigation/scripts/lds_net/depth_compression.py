"""Bounded S0-S3 depth surgery, training-only KD, and W4A6 recovery/QAT."""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
from ultralytics.models.yolo.detect import DetectionTrainer
from .data import ROOT, COCO, seg_loader, seed_worker, write_json
from .training import RoadTrainer, load_model
from .quant_training import QuantTrainer, QuantTrainingModel, CoreBranch, PairedRoadSampler
from .deploy_quant import QConv, QAdd, TensorQuant, observers, set_policy, calibrate
from .aggressive_data import DeterministicRoadDataset, reset_detection_epoch
from .aggressive_training import pad_detection_batch
from .ablation_metrics import SizeValidator, road_detailed
from .quant_evaluation import values, validation_ids, ATTR_ROOT
from .ps_postprocess import decode_detection

GROUPS = {
    'S0': {'drop_p4': False, 'drop_seg': False, 'convolutions': 37, 'GFLOPs': .5606496},
    'S1': {'drop_p4': True, 'drop_seg': False, 'convolutions': 35, 'GFLOPs': .5237856},
    'S2': {'drop_p4': False, 'drop_seg': True, 'convolutions': 36, 'GFLOPs': .5311584},
    'S3': {'drop_p4': True, 'drop_seg': True, 'convolutions': 34, 'GFLOPs': .4942944},
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reconnect(core):
    """Python quantizer links are not state_dict tensors; rebind after surgery."""
    known = {'input'}
    for row in core.nodes:
        assert set(row['inputs']) <= known, row
        if row['op'] == 'conv':
            object.__setattr__(core.ops[row['id']], 'input_quant', core.qspec(row['inputs'][0]))
        elif row['op'] == 'add':
            object.__setattr__(core.ops[row['id']], 'input_quants', [core.qspec(k) for k in row['inputs']])
        known.add(row['id'])
    assert set(core.outputs + core.feature_outputs) <= known


def surgery(core, group):
    """Physically remove modules and their graph nodes. Operates on a fresh copy."""
    config = GROUPS[group]
    sources = {r.get('source'): r['id'] for r in core.nodes}
    aliases, removed = {}, set()
    if config['drop_p4']:
        reduce, expand = sources['model.6.reduce.conv'], sources['model.6.expand.conv']
        add = next(r for r in core.nodes if r['op'] == 'add' and expand in r['inputs'])
        entry = next(k for k in add['inputs'] if k != expand)
        # Entry is the ReLU output of the P4 downsampling convolution.
        assert core.qspec(entry).unsigned
        aliases[add['id']] = entry
        removed.update((reduce, expand, add['id']))
    if config['drop_seg']:
        target = sources['seg_head.semantic_refine.2.conv']
        aliases[target] = next(r['inputs'][0] for r in core.nodes if r['id'] == target)
        removed.add(target)
    def resolve(key):
        while key in aliases:
            key = aliases[key]
        return key
    core.nodes = [{**r, 'inputs': [resolve(k) for k in r['inputs']]}
                  for r in core.nodes if r['id'] not in removed]
    core.feature_outputs = [resolve(k) for k in core.feature_outputs]
    core.outputs = [resolve(k) for k in core.outputs]
    for key in removed:
        del core.ops[key]
    reconnect(core)
    core.compression_group = group
    core.compression_removed_nodes = sorted(removed)
    return core


def set_fake_quant(model, enabled, observe=False):
    for module in model.modules():
        if isinstance(module, TensorQuant):
            module.enabled = enabled
            module.observe = enabled and observe and not module.fixed_input
        elif isinstance(module, QConv):
            module.enabled = enabled
            module.update_weights = enabled and observe


def initialize_qat(model, manifest, device='cuda', limit=None):
    """Calibrate a reloaded CPU checkpoint on an explicitly shared device."""
    from .quant_calibration import batches
    model.to(device).eval()
    set_fake_quant(model, True, False)
    set_policy(model.core, 'W4A6')
    actual_device = next(model.core.parameters()).device
    calibrate(model.core, lambda: batches(manifest, device=actual_device, limit=limit))
    by_source = {r.get('source'): r['id'] for r in model.core.nodes}
    for attr in ('cv2', 'cv3'):
        for i, branch in enumerate(getattr(model.model[-1], attr)):
            iq = model.core.qspec(model.core.feature_outputs[2+i])
            for j, module in enumerate(branch):
                source = f'model.12.one2one_{attr}.{i}.{j}'+('.conv' if j < 2 else '')
                match = model.core.ops[by_source[source]]
                object.__setattr__(module, 'input_quant', iq)
                module.bits = match.bits
                module.output_quant.bits = match.output_quant.bits
                module.output_quant.scale.copy_(match.output_quant.scale)
                module.initialize_weight_scale()
                iq = module.output_quant
    return model


def feature_loss(student, teacher):
    terms = []
    for s, t in zip(student, teacher):
        t = t.detach().float()
        rms = t.square().mean((2, 3), keepdim=True).sqrt()
        # A teacher ReLU channel can be identically zero. Do not amplify a
        # student's nonzero response by 1e6 in that channel after deleting a block.
        floor = .1*t.square().mean((1, 2, 3), keepdim=True).sqrt().clamp_min(1e-3)
        denominator = torch.maximum(rms, floor).clamp_min(1e-3)
        terms.append(((s.float()-t)/denominator).square().mean())
    return torch.stack(terms).mean()


def road_kd(student, teacher, mask, temperature=2.):
    s = F.interpolate(student.float(), mask.shape[-2:], mode='bilinear', align_corners=False)
    t = F.interpolate(teacher.detach().float(), mask.shape[-2:], mode='bilinear', align_corners=False)
    confidence, category = t.softmax(1).max(1)
    accepted = (mask != 255) & (category == mask) & (confidence >= .7)
    divergence = F.kl_div(F.log_softmax(s/temperature, 1), F.softmax(t/temperature, 1),
                          reduction='none').sum(1) * temperature**2
    terms = [divergence[accepted & (mask == k)].mean() for k in (0, 1, 2)
             if (accepted & (mask == k)).any()]
    return torch.stack(terms).mean() if terms else s.sum()*0


def kd_gain(global_epoch, batch, steps=428):
    return min(1., max(0., (global_epoch + batch/steps)/2.))


class CompressionTrainingModel(QuantTrainingModel):
    @classmethod
    def from_training(cls, original, group):
        model = copy.deepcopy(original).cpu().float()
        model.__class__ = cls
        surgery(model.core, group)
        for module in model.modules():
            if isinstance(module, CoreBranch):
                object.__setattr__(module, 'core', model.core)
        model.criterion = None
        model.kd_gain = 0.
        model.compression_group = group
        model.compression_mode = 'recovery'
        model.requires_grad_(True)
        object.__setattr__(model, '_teacher', None)
        object.__setattr__(model, '_det_features', None)
        return model

    def __getstate__(self):
        state = super().__getstate__()
        # Neither the teacher nor a live autograd graph enters EMA/checkpoints.
        state.pop('_teacher', None)
        state.pop('_det_features', None)
        return state

    def attach_teacher(self, teacher):
        object.__setattr__(self, '_teacher', teacher)

    def _predict_once(self, x, profile=False, embed=None):
        features = self.core.features(x)
        if self.training:
            object.__setattr__(self, '_det_features', features)
        result = self.model[-1](list(features[2:]))
        if self.training:
            return result
        # Official training/detach behavior, same stable PS decoding at validation.
        predictions = result[1]['one2one']
        b = x.shape[0]
        raw = tuple(t for start, length, side in ((0, 400, 20), (400, 100, 10))
                    for t in (predictions['boxes'][:, :, start:start+length].reshape(b, 4, side, side),
                              predictions['scores'][:, :, start:start+length].reshape(b, 10, side, side)))
        return decode_detection(raw), result[1]

    def loss(self, batch, preds=None):
        from .model import segmentation_loss
        from .road_loss import boundary_ce_loss
        if self.criterion is None:
            self.criterion = self.init_criterion()
        c = self.criterion
        c.final_o2m = c.o2m_copy = c.o2m = .3
        c.o2o = .7
        preds = self.predict(batch['img']) if preds is None else preds
        features = getattr(self, '_det_features', None)
        object.__setattr__(self, '_det_features', None)
        detection, items = c(preds, batch)
        total = detection.sum()
        zero = total.detach()*0
        ce = dice = band = fd = fr = kl = zero
        teacher = getattr(self, '_teacher', None)
        use_kd = self.training and torch.is_grad_enabled()
        if use_kd:
            assert teacher is not None and features is not None
            with torch.no_grad():
                target = teacher.features(batch['img'])
            fd = feature_loss(features[2:], target[2:])
        if 'seg_img' in batch:
            road_features = self.core.features(batch['seg_img'])
            logits = self.core.road(road_features)
            ce, dice = segmentation_loss(logits, batch['seg_mask'])
            band = boundary_ce_loss(logits, batch['seg_mask'], 2)
            total = total + .1*(ce+dice+.2*band)*len(batch['img'])
            if use_kd:
                with torch.no_grad():
                    target_features = teacher.features(batch['seg_img'])
                    target_logits = teacher.road(target_features)
                fr = feature_loss(road_features[2:3], target_features[2:3])
                kl = road_kd(logits, target_logits, batch['seg_mask'])
        if use_kd:
            total = total + self.kd_gain*(.1*fd+.01*fr+.05*kl)*len(batch['img'])
        if not torch.isfinite(total):
            raise FloatingPointError('Non-finite compression loss; no automatic recipe change')
        return total, {**items, 'road_ce': ce.detach(), 'road_dice': dice.detach(),
                       'road_boundary_ce': band.detach(), 'kd_feature_det': fd.detach(),
                       'kd_feature_road': fr.detach(), 'kd_road': kl.detach(),
                       'kd_gain': total.detach().new_tensor(self.kd_gain)}


def protected_metrics(report):
    counts = report['small_targets']
    small = [counts[k] for k in ('short_lt8', 'short_8_to16')]
    result = {'recall_short_lt16': sum(r['matched'] for r in small)/max(sum(r['targets'] for r in small), 1)}
    result.update({f'ap_class_{k}': float(report['detection_per_class'].get(str(k), 0.)) for k in (0, 1, 3)})
    return result


def compression_acceptance(report, baseline):
    current, old = values(report), values(baseline)
    deficits = {}
    for key, value in old.items():
        budget = .005 if key == 'map' else .01
        error = current[key]-value if key.endswith(('as_direct', 'as_alternative')) else value-current[key]
        deficits[key] = max(0., error-budget)
    cp, bp = protected_metrics(report), protected_metrics(baseline)
    for key in bp:
        deficits[key] = max(0., max(0., bp[key]-.01)-cp[key])
    return {'eligible': not any(deficits.values()), 'values': current,
            'baseline': old, 'protected': cp, 'protected_baseline': bp,
            'deficits': deficits, 'total_deficit': sum(deficits.values()),
            'reference': 'Original selected W4A6 integer reference; budgets never accumulate'}


def selection_rank(gate):
    v = gate['values']
    return (float(gate['eligible']), -gate['total_deficit'], v['map'],
            v['foreground_miou'], v['alternative_iou'], v['boundary_f1_2px'])


class CompressionTrainer(QuantTrainer):
    def __init__(self, *, mode, group, teacher, **kwargs):
        self.mode, self.group, self.teacher = mode, group, teacher
        super().__init__(phase=mode, **kwargs)
        self.add_callback('on_train_batch_start', self.before_batch)

    def build_optimizer(self, model, name='auto', lr=1e-5, momentum=.9, decay=.0005, iterations=1e5):
        groups = {}
        sources = {r['id']: r.get('source', '') for r in model.core.nodes}
        for name, parameter in model.named_parameters():
            if not parameter.requires_grad:
                continue
            source = sources.get(name.split('.')[2], '') if name.startswith('core.ops.') else ''
            role = 'road' if source.startswith('seg_head.') else (
                'backbone' if source.startswith('model.') and int(source.split('.')[1]) < 9 else 'detection')
            rate = (2e-5 if role == 'backbone' else 1e-4) if self.mode == 'recovery' else (
                1e-5 if role == 'backbone' else 3e-5)
            wd = 0. if name.endswith('bias') else .0005
            group = groups.setdefault((role, wd), {'params': [], 'lr': rate, 'weight_decay': wd,
                                                    'role': role, 'param_group': 'bias' if wd == 0 else 'weight'})
            group['params'].append(parameter)
        return torch.optim.AdamW(list(groups.values()), betas=(.9, .999), fused=self.device.type == 'cuda')

    def get_validator(self):
        return SizeValidator(self.test_loader, save_dir=self.save_dir,
                             args=copy.copy(self.args), _callbacks=self.callbacks)

    def audit_start(self, _):
        assert self.batch_size == 16 and len(self.train_loader) == 428
        assert not self.amp and not torch.backends.cuda.matmul.allow_tf32 and not torch.backends.cudnn.allow_tf32
        assert not any(isinstance(m, nn.BatchNorm2d) for m in self.model.modules())
        assert all(v is not None for v in (self.model.model[-1].cv2, self.model.model[-1].cv3))
        self.teacher.to(self.device).float().eval().requires_grad_(False)
        self.model.attach_teacher(self.teacher)
        self.model.compression_mode = self.mode
        if self.resume and self.best.exists():
            checkpoint = torch.load(self.best, map_location='cpu', weights_only=False)
            rank = checkpoint.get('selection_rank')
            self.best_rank = tuple(rank) if rank is not None else None
            self.best_epoch = checkpoint['selected_epoch']
        write_json(self.save_dir/'initialization.json', {
            'group': self.group, 'mode': self.mode, 'start_epoch': self.start_epoch+1,
            'epochs': self.epochs, 'resumed': bool(self.resume), 'batch': 16, 'road_batch': 16,
            'steps': 428, 'nbs': 64, 'BN_count': 0, 'AMP': False, 'TF32': False,
            'teacher_not_registered_in_model': '_teacher' not in self.model._modules,
            'optimizer': [{'role': g['role'], 'lr': g['initial_lr'], 'weight_decay': g['weight_decay']}
                          for g in self.optimizer.param_groups]})

    def sync_ema(self):
        ema = self.ema.ema
        ema.core.stage = self.model.core.stage
        for name, module in self.model.named_modules():
            if isinstance(module, TensorQuant):
                other = ema.get_submodule(name)
                other.bits, other.unsigned = module.bits, module.unsigned
                other.enabled, other.observe = module.enabled, False
                other.scale.copy_(module.scale)
            elif isinstance(module, QConv):
                other = ema.get_submodule(name)
                other.bits, other.enabled, other.update_weights = module.bits, module.enabled, False
                other.weight_scale.copy_(module.weight_scale)

    def epoch_start(self, trainer):
        RoadTrainer.epoch_start(self, trainer)
        global_epoch = self.epoch + (4 if self.mode == 'qat' else 0)
        reset_detection_epoch(self.train_loader, global_epoch)
        set_fake_quant(self.model, self.mode == 'qat', self.mode == 'qat' and self.epoch < 2)
        self.sync_ema()
        if self.road_train_loader is not None:
            self.road_train_loader.batch_sampler.set_epoch(global_epoch)
            self.road_iterator = iter(self.road_train_loader)

    def before_batch(self, _):
        assert self.batch_size == 16 and len(self.train_loader) == 428, 'No automatic batch reduction'
        epoch = self.epoch + (4 if self.mode == 'qat' else 0)
        self.model.kd_gain = kd_gain(epoch, self.epoch_batches)

    def preprocess_batch(self, batch):
        batch = pad_detection_batch(DetectionTrainer.preprocess_batch(self, batch), 16)
        if self.road_train_loader is None:
            sampler = PairedRoadSampler(8882, epoch=self.epoch+(4 if self.mode == 'qat' else 0))
            self.road_train_loader = DataLoader(DeterministicRoadDataset(), batch_sampler=sampler,
                num_workers=2, worker_init_fn=seed_worker, pin_memory=True)
            self.road_iterator = iter(self.road_train_loader)
        road = next(self.road_iterator)
        batch['seg_img'] = road['img'].to(self.device, non_blocking=True).float()/255
        batch['seg_mask'] = road['mask'].to(self.device, non_blocking=True)
        if self.epoch == 0 and self.epoch_batches < 2:
            records_path = self.save_dir/'paired_batch_hashes.json'
            records = json.loads(records_path.read_text()) if records_path.exists() else {}
            record = {'detection': hashlib.sha256(batch['img'].detach().cpu().numpy().tobytes()).hexdigest(),
                      'road': hashlib.sha256(batch['seg_img'].detach().cpu().numpy().tobytes()).hexdigest(),
                      'mask': hashlib.sha256(batch['seg_mask'].detach().cpu().numpy().tobytes()).hexdigest(),
                      'detection_files': [Path(p).name for p in batch['im_file']], 'road_ids': road['id']}
            records[str(self.epoch_batches)] = record
            write_json(records_path, records)
            control = self.experiment.parent/'S0/training'/self.mode/'paired_batch_hashes.json'
            if self.group != 'S0' and control.exists():
                assert json.loads(control.read_text())[str(self.epoch_batches)] == record, 'Paired data replay differs'
        return batch

    def publish_progress(self):
        status = self.experiment/'status.json'
        if status.exists():
            row = json.loads(status.read_text(encoding='utf-8'))
            revision = (row.get('time'), self.epoch, row.get('batch'))
            if revision == getattr(self, '_published_progress_revision', None):
                return
            try:
                write_json(self.experiment.parent/'status.json', {**row, 'group': self.group,
                    'stage': self.mode, 'global_epoch': self.epoch+1+(4 if self.mode == 'qat' else 0),
                    'total_group_epochs': 20, 'bit_stage': 'FP32 recovery' if self.mode == 'recovery' else 'W4A6',
                    'quant_scales_frozen': self.mode == 'qat' and self.epoch >= 2})
                self._published_progress_revision = revision
            except PermissionError as error:
                # A stale optional aggregate progress file must not abort SGD.
                print(f'PROGRESS_WRITE_DEFERRED {self.group}: {error}', flush=True)

    def batch_progress(self, trainer):
        RoadTrainer.batch_progress(self, trainer)
        self.publish_progress()

    def record_progress(self, trainer):
        RoadTrainer.record_progress(self, trainer)
        self.publish_progress()

    def validate(self):
        model = self.ema.ema
        set_fake_quant(model, self.mode == 'qat', False)
        self.validator.size_counts = {k: {'targets': 0, 'matched': 0}
                                     for k in ('short_lt8', 'short_8_to16', 'short_ge16')}
        metrics = self.validator(self)
        metrics.pop('fitness', None)
        if self.road_val_loader is None:
            self.road_val_loader = seg_loader('val', batch=16, workers=0, ids=validation_ids())
        attributes = json.loads((ATTR_ROOT/'attributes.json').read_text(encoding='utf-8'))
        road = road_detailed(model, attributes, device=str(self.device), loader=self.road_val_loader)
        report = {'detection': metrics, 'road': road, 'small_targets': self.validator.size_counts,
                  'detection_per_class': {str(k): float(v) for k, v in
                      zip(self.validator.metrics.box.ap_class_index, self.validator.metrics.box.ap)}}
        gate = compression_acceptance(report, self.baseline)
        rank = selection_rank(gate)
        selectable = self.mode == 'recovery' or self.epoch >= 2
        improved = selectable and (self.mode == 'recovery' or self.best_rank is None or rank > self.best_rank)
        if improved:
            self.best_rank, self.best_epoch = rank, self.epoch
            self.best_fitness = float(self.epoch+1)
        if self.best_fitness is None:
            self.best_fitness = -1000.
        fitness = self.best_fitness if improved else -float(self.epoch+1)
        self.latest_road = road
        metrics.update({'road/foreground_miou': road['foreground_miou'], 'road/iou_direct': road['iou'][1],
                        'road/iou_alternative': road['iou'][2], 'road/boundary_f1_2px_320': road['boundary']['2']['f1'],
                        'road/alt_as_direct': gate['values']['alternative_as_direct'],
                        'road/direct_as_alt': gate['values']['direct_as_alternative'],
                        'selection/eligible': float(gate['eligible'])})
        write_json(self.save_dir/'epoch_metrics'/f'{self.epoch+1:03d}.json', {
            'group': self.group, 'mode': self.mode, 'epoch': self.epoch+1,
            'selectable': selectable, 'new_best': improved, 'gate': gate, 'report': report})
        print(f'COMPRESSION_EPOCH {self.group} {self.mode} {self.epoch+1}/{self.epochs} '
              f'mAP={gate["values"]["map"]:.5f} road={road["foreground_miou"]:.5f} '
              f'eligible={gate["eligible"]}', flush=True)
        return metrics, fitness

    def resume_training(self, checkpoint):
        super().resume_training(checkpoint)
        if checkpoint and self.resume and checkpoint.get('scheduler_state'):
            self.scheduler.load_state_dict(checkpoint['scheduler_state'])

    def _handle_nan_recovery(self, epoch):
        if self.loss is not None and not torch.isfinite(self.loss).all():
            raise FloatingPointError('Non-finite loss; group fails without automatic recovery')
        if self.fitness is not None and not math.isfinite(float(self.fitness)):
            raise FloatingPointError('Non-finite fitness')
        return False


def train_stage(model, teacher, root, group, mode, baseline, manifest):
    out = Path(root)/group
    stage = out/'training'/mode
    epochs = 4 if mode == 'recovery' else 16
    last = stage/'weights/last.pt'
    completion = stage/'completion.json'
    if completion.exists():
        return load_model(last if mode == 'recovery' else stage/'weights/best.pt')
    model.requires_grad_(True)
    overrides = {'model': 'yolo26n.yaml', 'data': str(COCO), 'task': 'detect', 'pretrained': False,
        'epochs': epochs, 'patience': 0, 'imgsz': 320, 'batch': 16, 'nbs': 64, 'workers': 4, 'device': 0,
        'optimizer': 'AdamW', 'lr0': 1e-5, 'lrf': .1, 'weight_decay': .0005, 'cos_lr': True,
        'warmup_epochs': 0., 'amp': False, 'seed': 20261017, 'deterministic': True,
        'rect': False, 'multi_scale': 0., 'mosaic': 0., 'close_mosaic': 0, 'mixup': 0., 'cutmix': 0.,
        'scale': .3, 'translate': .1, 'fliplr': .5, 'save': True, 'save_period': 0, 'plots': True,
        'val': True, 'max_det': 300, 'nms': False, 'project': str(out/'training'), 'name': mode,
        'exist_ok': True, 'save_dir': str(stage), 'verbose': False}
    checkpoint = None
    if last.exists():
        checkpoint = torch.load(last, map_location='cpu', weights_only=False)
        if checkpoint['epoch']+1 >= epochs:
            return load_model(last if mode == 'recovery' else stage/'weights/best.pt')
        overrides.update(model=str(last), resume=str(last))
    trainer = CompressionTrainer(mode=mode, group=group, teacher=teacher, baseline=baseline,
        manifest=manifest, experiment=out, stage_name=mode, joint=True, overrides=overrides)
    trainer.model = checkpoint['model'] if checkpoint else model
    if checkpoint:
        original_resume = trainer.resume_training
        trainer.resume_training = lambda _: original_resume(checkpoint)
    try:
        trainer.train()
    finally:
        trainer.model.attach_teacher(None)
        for loader in (trainer.road_train_loader, trainer.road_val_loader):
            iterator = getattr(loader, '_iterator', None)
            if iterator is not None:
                iterator._shutdown_workers()
    return load_model(trainer.last if mode == 'recovery' else trainer.best)
