"""Immutable, training-only 512 COCO + 256 rich/256 other daytime BDD."""
import hashlib
import json
import random
from pathlib import Path
import cv2
import numpy as np
import torch
from .data import ROOT,SEG_ROOT,letterbox,write_json
from .quant_evaluation import ATTR_ROOT


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def make_calibration(directory, source_sha):
    directory=Path(directory); path=directory/'manifest.json'
    if path.exists():
        value=json.loads(path.read_text(encoding='utf-8'))
        assert value['source_sha256']==source_sha
        return value
    manifest=json.loads((SEG_ROOT/'manifest.json').read_text(encoding='utf-8'))
    attrs=json.loads((ATTR_ROOT/'attributes.json').read_text(encoding='utf-8'))
    daytime=[r for r in manifest['train'] if attrs.get(r['id'],{}).get('timeofday')=='daytime']
    a=sorted([r for r in daytime if r['pixels'][2]>=512],key=lambda r:r['id'])
    b=sorted([r for r in daytime if r['pixels'][2]<512],key=lambda r:r['id'])
    rng=random.Random(20261017)
    coco=sorted((ROOT/'data/coco_stuff_driving/images/train').glob('*.*'))
    selected=rng.sample(coco,512)
    rows=[{'id':p.stem,'dataset':'coco','pool':'train','path':str(p)} for p in selected]
    for pool,bank in (('daytime_alt_rich',a),('daytime_other',b)):
        rows += [{'id':r['id'],'dataset':'bdd','pool':pool,'path':r['cached_image'],
                  'mask_path':r['cached_mask']} for r in rng.sample(bank,256)]
    coco_val={p.stem for p in (ROOT/'data/coco_stuff_driving/images/val').glob('*.*')}
    road_val={r['id'] for r in manifest['val']}
    for row in rows:
        assert row['id'] not in (coco_val if row['dataset']=='coco' else road_val)
        row['sha256']=sha(row['path'])
        if 'mask_path' in row:
            row['mask_sha256']=sha(row['mask_path'])
    value={'seed':20261017,'source_sha256':source_sha,'images':rows,'count':1024,
           'counts':{'coco':512,'bdd_daytime_alt_rich':256,'bdd_daytime_other':256},
           'train_val_id_disjoint':True,'augmentation':False}
    write_json(path,value)
    return value


def batches(manifest, device='cuda', batch=16, limit=None):
    rows=manifest['images'][:limit]
    for start in range(0,len(rows),batch):
        images=[]
        for row in rows[start:start+batch]:
            image=cv2.imread(row['path'])
            if image is None:
                raise OSError(row['path'])
            if row['dataset']=='coco':
                image=letterbox(image)[0]
            assert image.shape==(320,320,3)
            images.append(np.ascontiguousarray(image[:,:,::-1].transpose(2,0,1)))
        yield torch.from_numpy(np.stack(images)).to(device).float()/255
