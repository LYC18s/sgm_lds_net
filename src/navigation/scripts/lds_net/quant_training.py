"""Official detached E2E assignment around the quantized single-pass core."""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
import torch
from torch import nn
from torch.utils.data import DataLoader,Sampler
from ultralytics.models.yolo.detect import DetectionTrainer
from ultralytics.nn.modules import Conv
from .model import LDSNetModel,fold_batchnorm
from .deploy_graph import ConvOp
from .deploy_quant import QConv,TensorQuant,observers,set_policy,calibrate,stage_bits
from .data import COCO,write_json,seg_loader,evaluate_segmentation,seed_worker
from .training import RoadTrainer,load_model
from .aggressive_data import DeterministicRoadDataset,reset_detection_epoch
from .aggressive_training import pad_detection_batch,road_statistics
from .quant_calibration import batches
from .quant_evaluation import acceptance,values


class CoreBranch(nn.Module):
    def __init__(self, core, scale, category):
        super().__init__()
        # Avoid registering all core parameters four more times in Detect.
        object.__setattr__(self,'core',core)
        self.scale,self.category=scale,category

    def forward(self,x):
        return self.core.branch(x,self.scale,self.category)


class QuantTrainingModel(LDSNetModel):
    def __init__(self, core, source):
        nn.Module.__init__(self)
        source=copy.deepcopy(source).float().eval()
        fold_batchnorm(source)
        head=source.model[-1]
        self.core=core
        self.model=nn.Sequential(head)
        self.seg_head=None # Custom loss below uses core.road, not a duplicate head.
        for category,attr in ((0,'one2one_cv2'),(1,'one2one_cv3')):
            setattr(head,attr,nn.ModuleList(CoreBranch(core,i,category) for i in range(2)))
        for attr,category in (('cv2',0),('cv3',1)):
            for i,branch in enumerate(getattr(head,attr)):
                iq=core.qspec(core.feature_outputs[2+i])
                layers=[]
                for j,old in enumerate(branch):
                    if isinstance(old,Conv):
                        original=ConvOp(old.conv,isinstance(old.act,nn.ReLU))
                    else:
                        original=ConvOp(old,False)
                    layer=QConv(original,iq,8,8)
                    # Initialize auxiliary ranges from the matching one-to-one
                    # path; they are training-only and observe during QAT.
                    source_name=f'model.12.one2one_{attr}.{i}.{j}'+('.conv' if isinstance(old,Conv) else '')
                    matched=next(r for r in core.nodes if r.get('source')==source_name)
                    layer.output_quant.scale.copy_(core.qspec(matched['id']).scale)
                    layers.append(layer);iq=layer.output_quant
                getattr(head,attr)[i]=nn.Sequential(*layers)
        self.stride=source.stride;self.names=source.names;self.nc=source.nc
        self.yaml=source.yaml;self.args=source.args;self.save=[];self.inplace=True;self.end2end=True
        self.criterion=None;self.seg_weight=.1;self.record_task_losses=True
        self.seg_boundary_weight=.2;self.seg_boundary_radius=2
        self.seg_aux_enabled=False;self.seg_da_enabled=False
        for p in self.parameters():p.requires_grad_(True)

    def features(self,x,shallow=False):
        f=self.core.features(x)
        return f[:2] if shallow else f

    def _predict_once(self,x,profile=False,embed=None):
        f=self.core.features(x)
        return self.model[-1](list(f[2:]))

    def predict_semantic(self,x):
        return self.core.road(self.core.features(x))

    def predict_both(self,x):
        f=self.core.features(x)
        return self.model[-1](list(f[2:])),self.core.road(f)

    def loss(self,batch,preds=None):
        from .model import segmentation_loss
        from .road_loss import boundary_ce_loss
        if self.criterion is None:self.criterion=self.init_criterion()
        c=self.criterion;c.final_o2m=c.o2m_copy=c.o2m=.3;c.o2o=.7
        preds=self.predict(batch['img']) if preds is None else preds
        det,items=c(preds,batch)
        total=det.sum()
        ce=dice=band=total.detach()*0
        if 'seg_img' in batch:
            logits=self.predict_semantic(batch['seg_img'])
            ce,dice=segmentation_loss(logits,batch['seg_mask'])
            band=boundary_ce_loss(logits,batch['seg_mask'],2)
            total=total+.1*(ce+dice+.2*band)*len(batch['img'])
        return total,{**items,'road_ce':ce.detach(),'road_dice':dice.detach(),'road_boundary_ce':band.detach()}

    def fuse(self,verbose=True,imgsz=320):
        # Validation never destroys auxiliary training branches. Deployment is
        # exported only from self.core, which cannot contain either branch.
        return self


class PairedRoadSampler(Sampler):
    def __init__(self,count,steps=428,batch=16,epoch=0):
        self.count,self.steps,self.batch,self.epoch=count,steps,batch,epoch
    def __len__(self):return self.steps
    def set_epoch(self,epoch):self.epoch=epoch
    def __iter__(self):
        import random
        rng=random.Random(20261017+self.epoch)
        order=list(range(self.count));rng.shuffle(order)
        # 6848 samples < 8882: uniform mixed road sampling, no hard pools.
        for step in range(self.steps):
            ids=order[step*self.batch:(step+1)*self.batch]
            assert len(ids)==self.batch
            yield [(i,rng.getrandbits(63)) for i in ids]


class QuantTrainer(RoadTrainer):
    def __init__(self,*,phase,baseline,manifest,**kwargs):
        self.phase=phase;self.baseline=baseline;self.manifest=manifest
        self.best_rank=None;self.best_epoch=-1;self.selected_epoch=None
        super().__init__(**kwargs)
        self.add_callback('on_train_start',self.audit_start)

    def build_optimizer(self,model,name='auto',lr=1e-5,momentum=.9,decay=1e-5,iterations=1e5):
        groups={}
        source_by_id={r['id']:r.get('source','') for r in model.core.nodes}
        for name,p in model.named_parameters():
            if not p.requires_grad:continue
            source=source_by_id.get(name.split('.')[2],'') if name.startswith('core.ops.') else ''
            role='road' if source.startswith('seg_head.') else ('backbone' if source.startswith('model.') and int(source.split('.')[1])<9 else 'detection')
            rate=1e-5 if role=='backbone' else 2e-5
            wd=0. if name.endswith('bias') else decay
            g=groups.setdefault((role,wd),{'params':[],'lr':rate,'weight_decay':wd,'role':role,
                                           'param_group':'bias' if wd==0 else 'weight'})
            g['params'].append(p)
        return torch.optim.AdamW(list(groups.values()),betas=(.9,.999),fused=self.device.type=='cuda')

    def audit_start(self,t):
        assert self.batch_size==16 and len(self.train_loader)==428
        assert not self.amp and not torch.backends.cuda.matmul.allow_tf32 and not torch.backends.cudnn.allow_tf32
        assert not any(isinstance(m,nn.BatchNorm2d) for m in self.model.modules())
        assert self.model.model[-1].cv2 is not None and self.model.model[-1].cv3 is not None
        if not self.resume:self.sync_aux_policy()
        if self.resume and self.best.exists():
            c=torch.load(self.best,map_location='cpu',weights_only=False)
            self.best_rank=tuple(c['selection_rank']);self.best_epoch=c['selected_epoch']
        write_json(self.save_dir/'initialization.json',{'phase':self.phase,'amp':False,'tf32':False,
            'BN_count':0,'auxiliary_branches':True,'batch':16,'road_batch':16,'steps':428,
            'nbs':64,'loss_gains':[.3,.7,.1],'optimizer':[
            {'role':g['role'],'lr':g['initial_lr'],'weight_decay':g['weight_decay']} for g in self.optimizer.param_groups]})

    def epoch_start(self,t):
        RoadTrainer.epoch_start(self,t)
        reset_detection_epoch(self.train_loader,self.epoch)
        if self.phase=='W4' and self.epoch==2 and self.model.core.stage!='W4A4':
            set_policy(self.model.core,'W4A4')
            calibrate(self.model.core,lambda:batches(self.manifest))
            self.sync_aux_policy()
        enabled=self.epoch<4 if self.phase in ('W4','rescue') else self.epoch<2
        observers(self.model.core,enabled)
        for m in self.model.model[-1].modules():
            if isinstance(m,TensorQuant):m.observe=enabled
            if isinstance(m,QConv):m.update_weights=enabled
        # EMA's Python bit-policy flags are not state_dict tensors.
        self.ema.ema.core.stage=self.model.core.stage
        for name,m in self.model.named_modules():
            if isinstance(m,TensorQuant):
                other=self.ema.ema.get_submodule(name)
                other.bits=m.bits;other.unsigned=m.unsigned;other.observe=False
            elif isinstance(m,QConv):
                self.ema.ema.get_submodule(name).bits=m.bits
        if self.road_train_loader is not None:
            self.road_train_loader.batch_sampler.set_epoch(self.epoch)
            self.road_iterator=iter(self.road_train_loader)

    def batch_progress(self,t):
        super().batch_progress(t)
        status_path=self.experiment/'status.json'
        if status_path.exists():
            row=json.loads(status_path.read_text(encoding='utf-8'))
            write_json(self.experiment.parent/'status.json',{**row,
                'stage':self.phase if self.phase.endswith('_QAT') else self.phase+'_QAT','bit_stage':self.model.core.stage,
                'quant_scales_frozen':not any(m.observe for m in self.model.core.modules() if isinstance(m,TensorQuant))})

    def sync_aux_policy(self):
        weight_bits,activation_bits=stage_bits(self.model.core.stage)
        for attr in ('cv2','cv3'):
            for branch in getattr(self.model.model[-1],attr):
                for j,m in enumerate(branch):
                    # Output projection remains W8/A8; preceding feature is A8.
                    oldmax=float(m.output_quant.scale)*m.output_quant.limits[1]
                    oldbits=m.output_quant.bits
                    old_weight_bits=m.bits
                    m.bits=8 if j==2 else weight_bits
                    if m.bits==2 and (old_weight_bits!=2 or not hasattr(m,'w2_scale_ratio')):
                        m.initialize_two_bit_ratio()
                    m.output_quant.bits=8 if j>=1 else activation_bits
                    if oldbits!=m.output_quant.bits:
                        m.output_quant.scale.fill_(max(oldmax/m.output_quant.limits[1],1e-8))
                    m.initialize_weight_scale()

    def preprocess_batch(self,batch):
        batch=pad_detection_batch(DetectionTrainer.preprocess_batch(self,batch),16)
        if self.road_train_loader is None:
            ds=DeterministicRoadDataset()
            sampler=PairedRoadSampler(len(ds),epoch=self.epoch)
            self.road_train_loader=DataLoader(ds,batch_sampler=sampler,num_workers=2,
                worker_init_fn=seed_worker,pin_memory=True)
            self.road_iterator=iter(self.road_train_loader)
        road=next(self.road_iterator)
        batch['seg_img']=road['img'].to(self.device,non_blocking=True).float()/255
        batch['seg_mask']=road['mask'].to(self.device,non_blocking=True)
        return batch

    def optimizer_step(self):
        super().optimizer_step()
        # EMA averaging quantization ranges would create a second, incompatible
        # quantizer. Copy ranges exactly; EMA averages only learned parameters.
        with torch.no_grad():
            live=self.model.state_dict()
            for name,v in self.ema.ema.state_dict().items():
                if name.endswith(('.scale','.weight_scale','.w2_scale_ratio')):
                    v.copy_(live[name])

    def validate(self):
        ema=self.ema.ema
        ema.seg_boundary_weight=.2;ema.seg_boundary_radius=2
        observers(ema.core,False)
        for m in ema.modules():
            if isinstance(m,TensorQuant):m.observe=False
            if isinstance(m,QConv):m.update_weights=False
        # Official validator calls loss with detached predictions too.
        ema.criterion=ema.init_criterion()
        c=ema.criterion;c.final_o2m=c.o2m_copy=c.o2m=.3;c.o2o=.7
        metrics=self.validator(self);metrics.pop('fitness',None)
        if self.road_val_loader is None:
            from .quant_evaluation import validation_ids
            self.road_val_loader=seg_loader('val',batch=16,workers=0,ids=validation_ids())
        road=evaluate_segmentation(ema,self.road_val_loader,self.device);self.latest_road=road
        metrics.update({k:v for k,v in road.items() if isinstance(v,(int,float))})
        metrics.update(road_statistics(road['road/confusion']))
        report={'detection':metrics,'road':{'confusion':road['road/confusion'],
            'iou':[road['road/iou_background'],road['road/iou_direct'],road['road/iou_alternative']],
            'foreground_miou':road['road/foreground_miou'],
            'boundary':{'2':{'f1':road['road/boundary_f1_2px_320']}}}}
        gate=acceptance(report,self.baseline)
        v=gate['values'];rank=(float(gate['eligible']),-gate['total_deficit'],v['foreground_miou'],v['map'])
        # W4A8 is a transition, never selected as a mixed-W4A4 checkpoint.
        selectable=(self.model.core.stage==self.target_stage if hasattr(self,'target_stage')
                    else self.phase!='W4' or self.epoch>=2)
        improved=selectable and (self.best_rank is None or rank>self.best_rank)
        if improved:self.best_rank=rank;self.best_epoch=self.epoch;self.best_fitness=float(self.epoch+1)
        if self.best_fitness is None:self.best_fitness=-1000.
        fitness=self.best_fitness if improved else -float(self.epoch+1)
        write_json(self.save_dir/'epoch_metrics'/f'{self.epoch+1:03d}.json',{'epoch':self.epoch+1,
             'stage':ema.core.stage,'gate':gate,'road':road,'metrics':metrics,'new_best':improved})
        if self.phase in ('W4','W4A6','W2A6') and self.epoch+1>=8 and self.best_epoch>=0 and self.epoch-self.best_epoch>=6:
            self.stop=True
        print(f'QUANT_EPOCH {self.phase} {self.epoch+1}/{self.epochs} '
              f'mAP={v["map"]:.5f} road={v["foreground_miou"]:.5f} eligible={gate["eligible"]}',flush=True)
        return metrics,fitness

    def save_model(self):
        # Unlike the engine default, retain FP32 parameters/scales and optimizer
        # state. Keep LIVE model for exact resume and EMA for candidate selection.
        live=copy.deepcopy(self.model).cpu().float();live.criterion=None
        ema=copy.deepcopy(self.ema.ema).cpu().float();ema.criterion=None
        checkpoint={'epoch':self.epoch,'best_fitness':self.best_fitness,'model':live,'ema':ema,
            'updates':self.ema.updates,'optimizer':self.optimizer.state_dict(),'scaler':self.scaler.state_dict(),
            'train_args':vars(self.args),'train_metrics':self.metrics,'train_results':self.read_results_csv(),
            'selection_rank':self.best_rank,'selected_epoch':self.best_epoch,'phase':self.phase,
            'modelopt':None,'version':'8.4.163','precision':'FP32',
            'scheduler_state':self.scheduler.state_dict()}
        self.wdir.mkdir(parents=True,exist_ok=True)
        torch.save(checkpoint,self.last)
        if self.fitness==self.best_fitness:torch.save(checkpoint,self.best)
        # Retain a W4A8 transition checkpoint separately.
        if self.phase=='W4' and self.epoch==1:torch.save(checkpoint,self.wdir/'W4A8_transition.pt')
        return True


def train_quant(core,source,root,phase,baseline,manifest,epochs,initial_training=None,initial_optimizer=None):
    out=Path(root)/phase
    last=out/'training/qat/weights/last.pt'
    complete=out/'training/qat/completion.json'
    if complete.exists():
        return load_model(out/'training/qat/weights/best.pt')
    if initial_training is None:
        model=QuantTrainingModel(core,source)
    else:
        model=copy.deepcopy(initial_training).cpu().float()
        model.core.load_state_dict(core.state_dict())
        model.core.stage=core.stage
        for name,m in core.named_modules():
            other=model.core.get_submodule(name)
            if isinstance(m,TensorQuant):other.bits=m.bits;other.unsigned=m.unsigned
            if isinstance(m,QConv):other.bits=m.bits
        model.criterion=None
    for p in model.parameters():p.requires_grad_(True)
    model.seg_weight=.1
    overrides={'model':'yolo26n.yaml','data':str(COCO),'task':'detect','pretrained':False,
        'epochs':epochs,'patience':0,'imgsz':320,'batch':16,'nbs':64,'workers':4,'device':0,
        'optimizer':'AdamW','lr0':1e-5,'lrf':.05,'weight_decay':.0005,'cos_lr':True,
        'warmup_epochs':0.,'amp':False,'seed':20261017,'deterministic':True,
        'rect':False,'multi_scale':0.,'mosaic':0.,'close_mosaic':0,'mixup':0.,'cutmix':0.,
        'scale':.3,'translate':.1,'fliplr':.5,'save':True,'save_period':0,'plots':True,
        'val':True,'max_det':300,'nms':False,'project':str(out/'training'),'name':'qat',
        'exist_ok':True,'save_dir':str(out/'training/qat'),'verbose':False}
    if last.exists():
        # Engine resume loads EMA by default. Supply the live model explicitly
        # after setup_model has read checkpoint metadata, preserving optimizer pairing.
        overrides.update(model=str(last),resume=str(last))
    trainer_class=QuantTrainer
    if phase in ('W4A6','W2A6'):
        from .a6_training import A6Trainer
        trainer_class=A6Trainer
    trainer=trainer_class(phase=phase,baseline=baseline,manifest=manifest,
        experiment=out,stage_name=phase,joint=True,overrides=overrides)
    if not last.exists():trainer.model=model
    else:
        checkpoint=torch.load(last,map_location='cpu',weights_only=False)
        if checkpoint['epoch']+1>=epochs:
            return load_model(out/'training/qat/weights/best.pt')
        trainer.model=checkpoint['model']
        # setup_model normally returns None when given nn.Module; restore resume
        # state in resume_training below via an explicit saved checkpoint.
        original_resume=trainer.resume_training
        def restore(_):original_resume(checkpoint)
        trainer.resume_training=restore
    if initial_optimizer is not None and not last.exists():
        def preserve_moments(t):
            rates=[(g['lr'],g['initial_lr']) for g in t.optimizer.param_groups]
            t.optimizer.load_state_dict(copy.deepcopy(initial_optimizer))
            for g,(lr,initial_lr) in zip(t.optimizer.param_groups,rates):
                g['lr']=lr;g['initial_lr']=initial_lr
            write_json(t.save_dir/'transition_state.json',{
                'auxiliary_weights_preserved':True,'optimizer_moments_preserved':True,
                'new_stage_cosine_schedule':True})
        trainer.add_callback('on_train_start',preserve_moments)
    try:
        trainer.train()
    finally:
        for loader in (trainer.road_train_loader,trainer.road_val_loader):
            iterator=getattr(loader,'_iterator',None)
            if iterator is not None:iterator._shutdown_workers()
    return load_model(trainer.best)
