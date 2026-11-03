"""Fixed-input diagnostics; retain the existing two-pixel boundary definition."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from ultralytics.utils.metrics import box_iou

from .data import ROOT, COCO, SEG_ROOT, seg_loader, write_json
from .model import RawExport
from .training import SquareValidator, load_model


def attribute_index():
    wanted = {r['id'] for s in ('train', 'val') for r in json.loads((SEG_ROOT/'manifest.json').read_text())[s]}
    attributes = {}
    for root in (ROOT/'data/bdd100k_detection_8k/original_labels', ROOT/'data/bdd9_scene_balanced'):
        for path in sorted(root.rglob('*.json')):
            if path.stem not in wanted:
                continue
            value = json.loads(path.read_text(encoding='utf-8'))
            if isinstance(value, list):
                value = value[0] if value else {}
            attr = value.get('attributes', {})
            if attr:
                attributes.setdefault(path.stem, {k: str(attr.get(k, 'unknown')) for k in ('timeofday','weather')})
    return attributes


def confusion_metrics(hist):
    h = np.asarray(hist, dtype=np.float64)
    tp = np.diag(h)
    def divide(a, b):
        return np.divide(a, b, out=np.zeros_like(a), where=b>0)
    union = h.sum(0)+h.sum(1)-tp
    iou = divide(tp, union)
    present = union[1:]>0
    return {'confusion': h.astype(np.int64).tolist(), 'iou': iou.tolist(),
            'precision': divide(tp,h.sum(0)).tolist(), 'recall': divide(tp,h.sum(1)).tolist(),
            'foreground_miou': float(iou[1:][present].mean()) if present.any() else None}


def boundary_counts(prediction, masks, tolerances=(2,4,8)):
    valid = masks != 255
    # Keep the original 3x3 valid-domain erosion and inner class boundaries.
    interior = -F.max_pool2d(-valid[:,None].float(),3,1,1) > .5
    counts = torch.zeros((len(tolerances),2,4),device=masks.device)
    for c in (1,2):
        target = (masks==c)[:,None].float()
        pred = (prediction==c)[:,None].float()
        true_edge = (target+F.max_pool2d(-target,3,1,1)>.5)&interior
        pred_edge = (pred+F.max_pool2d(-pred,3,1,1)>.5)&interior
        for i,t in enumerate(tolerances):
            near_true=F.max_pool2d(true_edge.float(),2*t+1,1,t)>.5
            near_pred=F.max_pool2d(pred_edge.float(),2*t+1,1,t)>.5
            counts[i,c-1]=torch.stack(((pred_edge&near_true).sum(),pred_edge.sum(),
                                     (true_edge&near_pred).sum(),true_edge.sum()))
    return counts


def boundary_f1(counts):
    a = np.asarray(counts,dtype=np.float64)
    precision = a[...,0]/np.maximum(a[...,1],1)
    recall = a[...,2]/np.maximum(a[...,3],1)
    return 2*precision*recall/np.maximum(precision+recall,1e-12)


@torch.inference_mode()
def road_detailed(model, attributes, device='cuda', loader=None):
    model = model.to(device).float().eval()
    loader = loader if loader is not None else seg_loader('val',workers=0)
    total=np.zeros((3,3),dtype=np.int64)
    edges=np.zeros((3,2,4),dtype=np.float64)
    groups={}; per_image=[]
    for batch in loader:
        x=batch['img'].to(device).float()/255
        mask=batch['mask'].to(device)
        pred=F.interpolate(model.predict_semantic(x).float(),mask.shape[-2:],mode='bilinear',align_corners=False).argmax(1)
        valid=mask!=255
        offsets=torch.arange(len(x),device=device)[:,None,None]*9
        histogram=torch.bincount((offsets+mask*3+pred)[valid],minlength=len(x)*9).reshape(len(x),3,3).cpu().numpy()
        edges+=boundary_counts(pred,mask).cpu().numpy()
        total+=histogram.sum(0)
        for identity,h in zip(batch['id'],histogram):
            attr=attributes.get(identity,{})
            for field in ('timeofday','weather'):
                key=field+'/'+attr.get(field,'unknown')
                g=groups.setdefault(key,{'images':0,'hist':np.zeros((3,3),dtype=np.int64)})
                g['images']+=1; g['hist']+=h
            per_image.append({'id':identity,'confusion':h.tolist()})
    return {**confusion_metrics(total),'images':len(per_image),
        'boundary': {str(t): {'f1':float(boundary_f1(edges[i].sum(0))),
                             'per_foreground_class_f1':boundary_f1(edges[i]).tolist()}
                     for i,t in enumerate((2,4,8))},
        'boundary_coordinate_space':'320x320 letterbox pixels; ignore255; original inner-boundary definition',
        'groups': {k:{'images':v['images'],**confusion_metrics(v['hist'])} for k,v in groups.items()},
        'per_image':per_image}


class SizeValidator(SquareValidator):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.size_counts={k:{'targets':0,'matched':0} for k in ('short_lt8','short_8_to16','short_ge16')}

    def update_metrics(self,preds,batch):
        for i,pred in enumerate(preds):
            gt=self._prepare_batch(i,batch)
            p=self._prepare_pred(pred)
            keep=p['conf']>=.25
            boxes=p['bboxes'][keep]; classes=p['cls'][keep]
            short=(gt['bboxes'][:,2:]-gt['bboxes'][:,:2]).amin(1).cpu().tolist()
            matches=[]
            if len(boxes) and len(short):
                iou=box_iou(gt['bboxes'],boxes)
                pairs=torch.nonzero((iou>=.5)&(gt['cls'][:,None]==classes[None,:]))
                matches=sorted([(int(g),int(p),float(iou[g,p])) for g,p in pairs],key=lambda v:-v[2])
            used_gt=set();used_pred=set()
            for g,p,_ in matches:
                if g not in used_gt and p not in used_pred:
                    used_gt.add(g);used_pred.add(p)
            for g,value in enumerate(short):
                key='short_lt8' if value<8 else 'short_8_to16' if value<16 else 'short_ge16'
                self.size_counts[key]['targets']+=1
                self.size_counts[key]['matched']+=int(g in used_gt)
        super().update_metrics(preds,batch)


def validate_detailed(weights,output,attributes,road_ids=None):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    model=load_model(weights)
    validator=SizeValidator(save_dir=output/'detection',args={
        'data':str(COCO),'imgsz':320,'batch':32,'workers':0,'device':'0',
        'rect':False,'nms':False,'plots':True,'quantize':32,'max_det':300,
        'project':str(output),'name':'detection','exist_ok':True})
    metrics={k:float(v) for k,v in validator(model=copy.deepcopy(model)).items()}
    b=validator.metrics.box
    # Match the original AutoBackend validation: move to CUDA before folding BN.
    # CPU vs CUDA folding changes a few near-tied pixel classifications.
    model=model.to('cuda').float().eval().fuse(verbose=False)
    report={'precision':'FP32','bn_fusion_device':'cuda','input':[320,320],'detection':metrics,
        'classes':[{'id':int(c),'name':model.names[int(c)],'ap50':float(b.ap50[i]),
                    'ap50_95':float(b.ap[i]),'recall':float(b.r[i])} for i,c in enumerate(b.ap_class_index)],
        'small_targets':{'definition':'shorter GT side in 320 input coordinates; greedy class-aware IoU>=0.5, conf>=0.25; recall, NOT COCO AP-small',
            'groups':{k:{**v,'recall':v['matched']/max(v['targets'],1)} for k,v in validator.size_counts.items()}},
        'road':road_detailed(model,attributes,loader=seg_loader('val',workers=0,ids=road_ids)),
        'road_evaluation':'daytime' if road_ids is not None else 'all',
        'road_evaluation_ids':sorted(road_ids) if road_ids is not None else None}
    if road_ids is not None:
        report['road_all']=road_detailed(model,attributes)
    write_json(output/'metrics.json',report)
    return report


def architecture(model):
    original=sum(p.numel() for p in model.parameters())
    model=copy.deepcopy(model).cpu().float().eval().fuse(verbose=False)
    macs=[0];shapes={}
    def hook(name):
        def capture(layer,inputs,output):
            macs[0]+=output.numel()*(layer.in_channels//layer.groups)*layer.kernel_size[0]*layer.kernel_size[1]
            shapes[name]=list(output.shape)
        return capture
    handles=[m.register_forward_hook(hook(n)) for n,m in model.named_modules() if isinstance(m,torch.nn.Conv2d)]
    with torch.inference_mode(): output=RawExport(model)(torch.zeros(1,3,320,320))
    for h in handles:h.remove()
    return {'train_parameters':original,'deployment_parameters':sum(p.numel() for p in model.parameters()),
            'conv_MACs':macs[0],'conv_GFLOPs':2*macs[0]/1e9,
            'outputs':[list(x.shape) for x in output],'conv_output_shapes':shapes,
            'extra_feature_buffers_int8_bytes':{'det_p3_project':51200 if getattr(model,'det_p3_project',None) is not None else 0,
                                                 'road_context_20x20':6400 if getattr(model.seg_head,'context',None) is not None else 0,
                                                 'semantic_refine_40x40x32':51200 if getattr(model.seg_head,'semantic_refine',None) is not None else 0,
                                                 'semantic_refine_residual_40x40x16':25600 if getattr(model.seg_head,'semantic_refine',None) is not None else 0},
            'hardware_note':'Tensor sizes only; streaming reuse/lifetimes and BRAM/FPS need hardware measurements.'}
