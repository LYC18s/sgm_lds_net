"""R0-R3 training, exact deterministic batches, and class-specific selection."""
import hashlib
import json

import numpy as np
import torch
from ultralytics.models.yolo.detect import DetectionTrainer

from .ablation_training import AblationTrainer
from .aggressive_data import road_training_loader,reset_detection_epoch
from .data import write_json,seg_loader,evaluate_segmentation
from .discrimination import discrimination_weight
from .loss_training import configure_loss
from .training import RoadTrainer,load_model


def configure_aggressive(model,group):
    configure_loss(model,1.,.2)
    model.seg_aux_enabled=False;model.seg_aux_weight=0.
    model.seg_da_enabled=group in ('R1','R3');model.seg_da_weight=0.;model.seg_da_seed=20261017
    if group in ('R2','R3'):model.seg_head.add_semantic_refinement()
    return model


def road_statistics(histogram):
    h=np.asarray(histogram,dtype=np.float64)
    return {'road/direct_as_alt':float(h[1,2]/max(h[1].sum(),1)),
        'road/alt_as_direct':float(h[2,1]/max(h[2].sum(),1)),
        **{f'road/{stat}_{name}':float(h[c,c]/max(h[:,c].sum() if stat=='precision' else h[c].sum(),1))
           for c,name in ((0,'background'),(1,'direct'),(2,'alternative')) for stat in ('precision','recall')}}


def selection(metrics,gates,gflops):
    values={'map':metrics['metrics/mAP50-95(B)'],'direct_iou':metrics['road/iou_direct'],
        'alt_iou':metrics['road/iou_alternative'],'alt_precision':metrics['road/precision_alternative'],
        'road':metrics['road/foreground_miou'],'boundary':metrics['road/boundary_f1_2px_320']}
    deficits={k:max(0.,gates[k]-v) for k,v in values.items()}
    deficits['alt_as_direct']=max(0.,metrics['road/alt_as_direct']-gates['alt_as_direct'])
    deficits['direct_as_alt']=max(0.,metrics['road/direct_as_alt']-gates['direct_as_alt'])
    deficits['gflops']=max(0.,gflops-.60)
    eligible=not any(deficits.values());violation=sum(deficits.values())
    quality=(values['alt_iou'],-metrics['road/alt_as_direct'],values['road'],values['map'],-gflops)
    key=(1.,)+quality if eligible else (0.,-violation)+quality
    return eligible,deficits,key


def validation_metrics(validation):
    r=validation['road']
    return {**validation['detection'],**road_statistics(r['confusion']),
        'road/iou_direct':r['iou'][1],'road/iou_alternative':r['iou'][2],
        'road/foreground_miou':r['foreground_miou'],'road/boundary_f1_2px_320':r['boundary']['2']['f1']}


def pad_detection_batch(batch,size):
    count=len(batch['img'])
    if count==size:return batch
    if not 0<count<size:raise ValueError('Unexpected detection batch size')
    repeat=[i%count for i in range(size-count)]
    old_indices=batch['batch_idx'].clone()
    extra_cls=[];extra_boxes=[];extra_indices=[]
    for offset,index in enumerate(repeat):
        selected=old_indices==index
        extra_cls.append(batch['cls'][selected]);extra_boxes.append(batch['bboxes'][selected])
        extra_indices.append(torch.full_like(old_indices[selected],count+offset))
    batch['img']=torch.cat((batch['img'],batch['img'][repeat]),0)
    for key,parts in (('cls',extra_cls),('bboxes',extra_boxes),('batch_idx',extra_indices)):
        batch[key]=torch.cat([batch[key]]+parts,0)
    for key,value in list(batch.items()):
        if isinstance(value,(list,tuple)) and len(value)==count:
            batch[key]=list(value)+[value[i] for i in repeat]
    return batch


class AggressiveTrainer(AblationTrainer):
    allowed_groups=['R0','R1','R2','R3']

    def __init__(self,**kwargs):
        self.best_rank=None
        super().__init__(**kwargs)
        self.protocol=json.loads((self.root/'protocol.json').read_text())
        self.bank=json.loads((self.root/'hard_pool.json').read_text())
        self.gflops=json.loads((self.experiment/'architecture.json').read_text())['conv_GFLOPs']
        self.add_callback('on_train_batch_start',self.schedule_discrimination)
        self.add_callback('on_train_batch_end',self.check_finite)

    def epoch_start(self,t):
        RoadTrainer.epoch_start(self,t)
        reset_detection_epoch(self.train_loader,self.epoch)
        if self.road_train_loader is not None:
            self.road_train_loader.batch_sampler.set_epoch(self.epoch)
            self.road_iterator=iter(self.road_train_loader)

    def schedule_discrimination(self,t):
        self.model.seg_da_weight=discrimination_weight(self.adapt,self.epoch,self.epoch_batches,
            len(self.train_loader),self.model.seg_da_enabled)
        offset=0 if self.adapt else self.protocol['adapt_epochs']*len(self.train_loader)
        self.model.seg_da_seed=20261017+offset+self.epoch*len(self.train_loader)+self.epoch_batches

    def check_finite(self,t):
        assert torch.isfinite(self.loss).all(),'Non-finite training loss; no silent recipe adjustment'

    def audit_start(self,t):
        super().audit_start(t)
        assert len(self.train_loader)==self.protocol['steps_per_epoch']
        expected_batch=4 if self.protocol['smoke_test_only'] else 32
        assert self.batch_size==expected_batch,'Automatic batch reduction invalidated paired experiment'
        if not self.resume:
            source=self.root/'reference/source.pt' if self.adapt else self.experiment/'training/adapt320/weights/last.pt'
            expected=load_model(source).state_dict();actual=self.model.state_dict()
            assert all(k in actual and torch.equal(v,actual[k].detach().cpu()) for k,v in expected.items())
            assert all(k in expected or k.startswith('seg_head.semantic_refine.') for k in actual)
        if self.resume and self.best.exists():
            checkpoint=torch.load(self.best,map_location='cpu',weights_only=False)
            self.best_rank=selection(checkpoint['train_metrics'],self.gates,self.gflops)[2]
        path=self.experiment/f'{self.stage_name}_initialization.json'
        record=json.loads(path.read_text());record.update({'source_weights_verified':not bool(self.resume),
            'aggressive_recipe':self.model.seg_da_enabled,'enhanced_semantics':self.group in ('R2','R3'),
            'road_batches':32,'steps_per_epoch':len(self.train_loader),
            'selection':'Lexicographic class-specific rank; fitness is checkpoint-choice epoch ordinal, not accuracy'})
        write_json(path,record)

    def preprocess_batch(self,batch):
        expected=4 if self.protocol['smoke_test_only'] else 32
        assert self.batch_size==expected
        batch=pad_detection_batch(DetectionTrainer.preprocess_batch(self,batch),expected)
        if self.road_train_loader is None:
            self.road_train_loader=road_training_loader(self.bank,self.group in ('R1','R3'),
                steps=len(self.train_loader),workers=0 if self.protocol['smoke_test_only'] else 2)
            self.road_train_loader.batch_sampler.set_epoch(self.epoch)
            self.road_iterator=iter(self.road_train_loader)
        road=next(self.road_iterator)
        batch['seg_img']=road['img'].to(self.device,non_blocking=True).float()/255
        batch['seg_mask']=road['mask'].to(self.device,non_blocking=True)
        assert len(road['id'])==32
        if self.epoch_batches<2:
            def digest(tensor):return hashlib.sha256(tensor.detach().cpu().contiguous().numpy().tobytes()).hexdigest()
            record={'epoch':self.epoch+1,'batch':self.epoch_batches+1,'detection_images':batch['im_file'],
                'detection_image_sha256':digest(batch['img']),
                'road_ids':road['id'],'road_image_sha256':digest(batch['seg_img']),
                'road_mask_sha256':digest(batch['seg_mask']),'da_seed':self.model.seg_da_seed}
            with (self.experiment/f'{self.stage_name}_batch_proofs.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(record)+'\n')
        return batch

    def validate(self):
        ema=self.ema.ema
        ema.seg_da_weight=self.model.seg_da_weight
        if ema.criterion is None:ema.criterion=ema.init_criterion()
        c=ema.criterion;c.final_o2m=c.o2m_copy=c.o2m=.3;c.o2o=.7
        metrics=self.validator(self);metrics.pop('fitness',None)
        if self.road_val_loader is None:self.road_val_loader=seg_loader('val',batch=32,workers=2,ids=self.road_ids)
        road=evaluate_segmentation(ema,self.road_val_loader,self.device);self.latest_road=road
        metrics.update({k:v for k,v in road.items() if isinstance(v,(int,float))})
        metrics.update(road_statistics(road['road/confusion']))
        raw=road['road/ce']+road['road/dice']+.2*road['road/boundary_ce']
        da=road['road/da_loss'];raw+=self.model.seg_da_weight*da
        metrics.update({'val/road_ce':road['road/ce'],'val/road_dice':road['road/dice'],
            'val/road_boundary_ce':road['road/boundary_ce'],'val/road_da':da,
            'recipe/da_weight':self.model.seg_da_weight,'val/road_raw':raw,'val/road_weighted':.1*raw})
        eligible,deficits,key=selection(metrics,self.gates,self.gflops)
        improved=self.best_rank is None or key>self.best_rank
        if improved:
            self.best_rank=key;self.best_fitness=float(self.epoch+1)
        fitness=self.best_fitness if improved else -float(self.epoch+1)
        metrics['selection/eligible']=float(eligible);metrics['selection/violation']=sum(deficits.values())
        write_json(self.experiment/'epoch_metrics'/f'{self.stage_name}_{self.epoch+1:03d}.json',
            {'epoch':self.epoch+1,'stage':self.stage_name,'metrics':metrics,'road':road,
             'selection_rank':key,'eligible':eligible,'deficits':deficits})
        write_json(self.experiment/f'{self.stage_name}_selection.json',{'epoch':self.epoch+1,'rank':key,
            'best_rank':self.best_rank,'eligible':eligible,'deficits':deficits,'new_best':improved})
        print(f'R_EXPERIMENT {self.group} {self.stage_name} epoch={self.epoch+1} '
            f'alt_iou={road["road/iou_alternative"]:.5f} alt_as_direct={metrics["road/alt_as_direct"]:.5f} eligible={eligible}',flush=True)
        return metrics,fitness
