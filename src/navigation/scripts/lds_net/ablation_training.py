"""A0-A3 controlled adaptation/joint training; no later phases are scheduled."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import torch

from .data import COCO, write_json
from .training import RoadTrainer, load_model, evaluate_segmentation, seg_loader


def configure_model(model, balanced):
    model.seg_weight=.1 if balanced else .5
    model.record_task_losses=True
    return model


def parameter_role(name):
    if name.startswith(('seg_head.','boundary_head.')):
        return 'road'
    if name.startswith('model.') and int(name.split('.')[1])<9:
        return 'backbone'
    return 'detection'


def freeze_statistics(model,adapt,balanced):
    for name,module in model.named_modules():
        if isinstance(module,torch.nn.BatchNorm2d):
            old_detector=name.startswith('model.')
            old_neck=old_detector and int(name.split('.')[1])<12
            if (adapt and old_detector) or (balanced and not adapt and
                    (old_neck or name.startswith('det_p3_project.'))):
                module.eval()


def selection_values(det,road,boundary,gates):
    violation=sum(max(0.,gates[k]-v) for k,v in zip(('map','road','boundary'),(det,road,boundary)))
    eligible=violation==0
    score=1+road+det*1e-6+boundary*1e-9 if eligible else -violation+road*1e-6
    return eligible,violation,score


class AblationTrainer(RoadTrainer):
    def __init__(self,*,balanced,adapt,gates,root,group,road_ids=None,**kwargs):
        self.balanced=balanced; self.adapt=adapt; self.gates=gates
        self.root=Path(root); self.group=group
        self.road_ids=road_ids
        self.frozen_state={}
        self.last_progress_mtime=None
        super().__init__(**kwargs)
        self.add_callback('on_train_start',self.audit_start)
        self.add_callback('on_fit_epoch_end',self.audit_epoch)
        self.add_callback('on_train_batch_end',self.mirror_progress)
        self.add_callback('on_fit_epoch_end',self.mirror_progress)

    def _model_train(self):
        super()._model_train()
        freeze_statistics(self.model,self.adapt,self.balanced)

    def build_optimizer(self,model,name='auto',lr=.001,momentum=.9,decay=1e-5,iterations=1e5):
        groups={}
        for module_name,module in model.named_modules():
            for suffix,param in module.named_parameters(recurse=False):
                if not param.requires_grad:
                    continue
                fullname=f'{module_name}.{suffix}'
                role=parameter_role(fullname)
                is_bias=suffix=='bias'
                wd=0. if is_bias or isinstance(module,torch.nn.BatchNorm2d) else decay
                if self.adapt:
                    rate=.001 if role=='road' else .00005
                elif self.balanced:
                    rate={'backbone':.00001,'detection':.00005,'road':.0003}[role]
                else:
                    rate=.00005
                key=(role,wd,is_bias)
                group=groups.setdefault(key,{'params':[],'lr':rate,'weight_decay':wd,
                    'param_group':'bias' if is_bias else 'weight','role':role})
                group['params'].append(param)
        optimizer=torch.optim.AdamW(list(groups.values()),betas=(momentum,.999),fused=self.device.type=='cuda')
        return optimizer

    def audit_start(self,t):
        self.model.criterion=self.model.init_criterion()
        c=self.model.criterion
        c.final_o2m=c.o2m_copy=c.o2m=.3 if self.balanced else .1
        c.o2o=1-c.o2m
        self._model_train()
        for name,value in self.model.state_dict().items():
            frozen_param=self.adapt and name.startswith('model.')
            bn_stat=any(name.endswith('.'+s) for s in ('running_mean','running_var','num_batches_tracked'))
            module_name=name.rsplit('.',1)[0]
            module=self.model.get_submodule(module_name) if bn_stat else None
            if frozen_param or (bn_stat and not module.training):
                self.frozen_state[name]=value.detach().cpu().clone()
        if self.resume and self.best.exists():
            b=torch.load(self.best,map_location='cpu',weights_only=False)
            self.stopper.best_fitness=float(b['best_fitness'])
            self.stopper.best_epoch=b['epoch']+1
        write_json(self.experiment/f'{self.stage_name}_initialization.json',{
            'group':self.group,'adapt':self.adapt,'balanced':self.balanced,
            'seg_weight':self.model.seg_weight,'head_gains':[c.o2m,c.o2o],
            'road_evaluation':'daytime' if self.road_ids is not None else 'all',
            'road_evaluation_images':len(self.road_ids) if self.road_ids is not None else 1000,
            'start_epoch':self.start_epoch+1,'epochs':self.epochs,
            'optimizer_groups':[{'role':g['role'],'lr':g['initial_lr'],
                'weight_decay':g['weight_decay'],'parameters':sum(p.numel() for p in g['params'])}
                for g in self.optimizer.param_groups],
            'frozen_tensors':len(self.frozen_state),'resumed':bool(self.resume)})

    def audit_epoch(self,t):
        assert len(self.optimizer.state)>0, 'No optimizer update occurred in this stage'
        state=self.model.state_dict()
        for name,value in self.frozen_state.items():
            assert torch.equal(state[name].detach().cpu(),value), f'Frozen parameter/statistic changed: {name}'
        write_json(self.experiment/f'{self.stage_name}_freeze_audit.json',{
            'epoch':self.epoch+1,'passed':True,'checked_tensors':len(self.frozen_state)})

    def mirror_progress(self,t):
        path=self.experiment/'status.json'
        if path.exists():
            mtime=path.stat().st_mtime_ns
            if self.last_progress_mtime==mtime:return
            self.last_progress_mtime=mtime
            status=json.loads(path.read_text())
            write_json(self.root/'status.json',{**status,'group':self.group,
                       'phase':'adapt' if self.adapt else 'joint',
                       'allowed_groups':getattr(self,'allowed_groups',['A0','A1','A2','A3'])})

    def validate(self):
        # EMA is a separate model; match its logging loss gains to the live model.
        ema=self.ema.ema
        if ema.criterion is None:
            ema.criterion=ema.init_criterion()
        c=ema.criterion
        c.final_o2m=c.o2m_copy=c.o2m=.3 if self.balanced else .1
        c.o2o=1-c.o2m
        metrics=self.validator(self)
        det=float(metrics.pop('fitness',metrics['metrics/mAP50-95(B)']))
        if self.road_val_loader is None:
            self.road_val_loader=seg_loader('val',batch=32,workers=2,ids=self.road_ids)
        road=evaluate_segmentation(self.ema.ema,self.road_val_loader,self.device)
        self.latest_road=road
        metrics.update({k:v for k,v in road.items() if isinstance(v,(int,float))})
        metrics['val/road_ce']=road['road/ce'];metrics['val/road_dice']=road['road/dice']
        metrics['val/road_raw']=road['road/ce']+road['road/dice']
        metrics['val/road_weighted']=self.model.seg_weight*metrics['val/road_raw']
        eligible,violation,fitness=selection_values(det,road['road/foreground_miou'],road['road/boundary_f1_2px_320'],self.gates)
        if self.adapt:
            fitness=road['road/foreground_miou']
        metrics['selection/eligible']=float(eligible);metrics['selection/violation']=violation
        if self.best_fitness is None or fitness>self.best_fitness:
            self.best_fitness=fitness
        print(f'ABLATION {self.group} {self.stage_name} epoch={self.epoch+1} map={det:.5f} '
              f'road={road["road/foreground_miou"]:.5f} boundary={road["road/boundary_f1_2px_320"]:.5f} eligible={eligible}',flush=True)
        return metrics,fitness


def fit_stage(model,root,group,adapt,epochs,gates,resume=False,smoke=False,road_ids=None,trainer_class=AblationTrainer):
    out=Path(root)/group;out.mkdir(exist_ok=True)
    stage='adapt320' if adapt else 'joint320'
    directory=out/'training'/stage
    if (directory/'completion.json').exists():
        return load_model(directory/'weights/last.pt' if adapt else directory/'weights/best.pt')
    last=directory/'weights/last.pt'
    do_resume=resume and last.exists()
    if do_resume:
        c=torch.load(last,map_location='cpu',weights_only=False)
        if c['epoch']+1>=epochs:
            write_json(directory/'completion.json',{'epochs_completed':epochs,'recovered_completed_checkpoint':True})
            return load_model(last if adapt else directory/'weights/best.pt')
    balanced=group!='A0'
    model=configure_model(model,balanced)
    for p in model.parameters():p.requires_grad_(True)
    overrides={'model':str(last) if do_resume else 'yolo26n.yaml',
        'data':str(COCO),'task':'detect','pretrained':False,'epochs':epochs,'patience':0,
        'imgsz':320,'batch':4 if smoke else 32,'nbs':4 if smoke else 64,
        'workers':0 if smoke else 4,'device':0,
        'optimizer':'AdamW','lr0':.00005,'lrf':.05,'weight_decay':.0005,'cos_lr':True,
        'warmup_epochs':0. if adapt else 1.,'warmup_bias_lr':.00001 if balanced else .00005,
        'amp':'bf16','seed':20261017,'deterministic':True,'rect':False,'multi_scale':0.,
        'mosaic':0.,'close_mosaic':0,'mixup':0.,'cutmix':0.,'scale':.3,'translate':.1,'fliplr':.5,
        'freeze':list(range(13)) if adapt else None,'save':True,'save_period':0,
        'plots':True,'val':True,'max_det':300,'nms':False,'fraction':.003 if smoke else 1.,
        'project':str(out/'training'),'name':stage,'exist_ok':True,'save_dir':str(directory),'verbose':False}
    if do_resume:overrides['resume']=str(last)
    trainer=trainer_class(balanced=balanced,adapt=adapt,gates=gates,root=root,group=group,
        road_ids=road_ids,experiment=out,stage_name=stage,joint=True,overrides=overrides)
    if not do_resume:trainer.model=model
    try:
        trainer.train()
    finally:
        for loader in (trainer.road_train_loader,trainer.road_val_loader):
            iterator=getattr(loader,'_iterator',None)
            if iterator is not None:iterator._shutdown_workers()
    return load_model(trainer.last if adapt else trainer.best)
