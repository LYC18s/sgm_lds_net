"""Identical PS decoding and fixed validation split for float, fake and integer."""
from __future__ import annotations
import copy
import json
import time
from pathlib import Path
import numpy as np
import torch
from torch import nn
from .ps_postprocess import decode_detection
from .integer_reference import IntegerReference
from .ablation_metrics import SizeValidator, road_detailed
from .data import ROOT, COCO, seg_loader, write_json

ATTR_ROOT = ROOT/'data/evaluation'


def validation_ids():
    value = json.loads((ATTR_ROOT/'road_evaluation.json').read_text(encoding='utf-8'))
    if isinstance(value,list):
        return value
    for k in ('ids','daytime_ids','road_ids','validation_ids'):
        if k in value:
            return value[k]
    raise ValueError(f'Unknown evaluation manifest fields: {value.keys()}')


class EvaluationAdapter(nn.Module):
    def __init__(self, core, names, integer_directory=None, cache=None):
        super().__init__()
        self.core = core
        self.names = names
        self.stride = torch.tensor([16.,32.])
        self.end2end = True
        self.yaml = {'channels':3}
        self.integer_directory = str(integer_directory) if integer_directory else None
        self.reference = None
        if integer_directory:
            spec = json.loads((Path(integer_directory)/'graph.json').read_text(encoding='utf-8'))
            if spec.get('format') == 'lds_net.integer.sparse.v1':
                from .sparse_weights import SparseIntegerReference
                self.reference = SparseIntegerReference(integer_directory)
            else:
                self.reference = IntegerReference(integer_directory)
        self.cache = Path(cache) if cache else None
        self.count = 0
        self.max_fake_integer_lsb = [0.]*5

    def raw(self, x):
        if self.reference is None:
            return self.core(x)
        result = []
        for image in x:
            # AutoBackend warmup supplies torch.empty(), not a valid RGB frame.
            # Encode the fixed INT8 interface by saturation, never int8 wrapping,
            # and compare both implementations on the SAME encoded pixel grid.
            pixels=torch.nan_to_num(image.detach().float().cpu()*255,
                                    nan=0.,posinf=255.,neginf=0.).round().clamp(0,255)
            q = (pixels.short()-128).to(torch.int8)[None]
            # Input content hash permits reuse across detection/road/reruns safely.
            import hashlib
            digest = hashlib.sha256(q.numpy().tobytes()).hexdigest()
            p = self.cache/(digest+'.npz') if self.cache else None
            if p and p.exists():
                with np.load(p) as a:
                    out = tuple(torch.from_numpy(a[f'output_{i}'].copy()) for i in range(5))
            else:
                out = self.reference(q)
                if p:
                    p.parent.mkdir(parents=True,exist_ok=True)
                    np.savez(p, **{f'output_{i}':v.numpy() for i,v in enumerate(out)})
            # Verify every validation image, including reused cached integer
            # outputs. An unexplained >1 LSB difference blocks publication.
            with torch.no_grad():
                canonical=(q.to(device=image.device).float()+128)/255
                fake=self.core(canonical)
            by_id={r['id']:r for r in self.reference.spec['nodes']}
            for i,(f,v,key) in enumerate(zip(fake,out,self.reference.spec['outputs'])):
                scale=by_id[key]['output_scale']
                difference=float((f.cpu()/scale-v.float()).abs().max())
                self.max_fake_integer_lsb[i]=max(self.max_fake_integer_lsb[i],difference)
                if difference>1.001:
                    raise AssertionError(f'Integer/fake mismatch {digest} output{i}: {difference} LSB')
            result.append(self.reference.dequantize(out))
            self.count += 1
            if self.count % 32 == 0:
                print(f'INTEGER_VALIDATION images={self.count}',flush=True)
        return tuple(torch.cat([v[i] for v in result]).to(x.device) for i in range(5))

    def forward(self, x):
        return decode_detection(self.raw(x))

    def predict_semantic(self, x):
        return self.raw(x)[-1]


def values(report):
    r = report['road']; h=np.asarray(r['confusion'],dtype=float)
    return {'map': report['detection']['metrics/mAP50-95(B)'],
            'foreground_miou':r['foreground_miou'],'direct_iou':r['iou'][1],
            'alternative_iou':r['iou'][2],'boundary_f1_2px':r['boundary']['2']['f1'],
            'alternative_as_direct':float(h[2,1]/max(h[2].sum(),1)),
            'direct_as_alternative':float(h[1,2]/max(h[1].sum(),1))}


def acceptance(candidate, baseline):
    c,b=values(candidate),values(baseline)
    deficits={}
    for k in b:
        budget=.01 if k=='map' else .02
        delta=c[k]-b[k] if k.endswith(('as_direct','as_alternative')) else b[k]-c[k]
        deficits[k]=max(0.,delta-budget)
    return {'eligible':not any(v>1e-12 for v in deficits.values()),
            'values':c,'baseline':b,'deficits':deficits,
            'total_deficit':sum(deficits.values())}


@torch.inference_mode()
def evaluate(core, names, output, integer_directory=None):
    output=Path(output)
    existing=output/'validation.json'
    if existing.exists():
        return json.loads(existing.read_text(encoding='utf-8'))
    output.mkdir(parents=True,exist_ok=True)
    model=EvaluationAdapter(copy.deepcopy(core).cuda().float().eval(), names, integer_directory,
                            output/'integer_cache' if integer_directory else None).eval()
    validator=SizeValidator(save_dir=output/'detection',args={
        'data':str(COCO),'imgsz':320,'batch':16,'workers':0,'device':'0',
        'rect':False,'nms':False,'plots':True,'quantize':32,'max_det':300,
        'project':str(output),'name':'detection','exist_ok':True})
    metrics={k:float(v) for k,v in validator(model=model).items()}
    attrs=json.loads((ATTR_ROOT/'attributes.json').read_text(encoding='utf-8'))
    road=road_detailed(model,attrs,loader=seg_loader('val',batch=16,workers=0,ids=validation_ids()))
    result={'precision':'integer-reference' if integer_directory else 'FP32-arithmetic',
        'detection':metrics,'road':road,'small_targets':validator.size_counts,
        'detection_per_class':{str(k):float(v) for k,v in zip(validator.metrics.box.ap_class_index,
                                                              validator.metrics.box.ap)},
        'road_images':576,'det_images':354,'nms':False,'stable_topk':True}
    if integer_directory:
        result['fake_integer_max_difference_lsb']=model.max_fake_integer_lsb
    write_json(existing,result)
    return result
