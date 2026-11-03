"""Deterministic full road batches and training-only C0 hard-example mining."""
import hashlib
import json
import math
import random

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

from .data import DrivableDataset,SEG_ROOT,seg_loader,seed_worker,write_json
from .training import load_model
from .discrimination import interior_regions


def reset_detection_epoch(loader,epoch):
    """Replay shuffling and worker augmentation after an epoch-boundary resume."""
    loader.reset()
    seed=20261017+int(epoch)
    loader.generator.manual_seed(seed)
    generator=getattr(loader.sampler,'generator',None)
    if generator is not None:generator.manual_seed(seed)
    random.seed(seed);np.random.seed(seed);torch.manual_seed(seed)


class DeterministicRoadDataset(DrivableDataset):
    def __init__(self):super().__init__('train',augment=False)

    def __getitem__(self,key):
        index,seed=key
        sample=super().__getitem__(index)
        rng=random.Random(seed)
        image=sample['img'].numpy();mask=sample['mask'].numpy()
        if rng.random()<.5:image=image[:,:,::-1];mask=mask[:,::-1]
        if rng.random()<.7:
            image=np.clip(image.astype(np.float32)*rng.uniform(.8,1.2)+rng.uniform(-12,12),0,255).astype(np.uint8)
        return {'img':torch.from_numpy(np.ascontiguousarray(image)),
            'mask':torch.from_numpy(np.ascontiguousarray(mask)),'id':sample['id']}


class RoadBatchSampler:
    def __init__(self,rows,bank,aggressive,steps=214):
        self.rows=rows;self.aggressive=aggressive;self.steps=steps;self.epoch=0
        lookup={r['id']:i for i,r in enumerate(rows)}
        self.a=[lookup[k] for k in bank['pool_A']];self.b=[lookup[k] for k in bank['pool_B']]
        self.h=[lookup[k] for k in bank['pool_H']]

    def __len__(self):return self.steps

    def set_epoch(self,epoch):self.epoch=epoch

    def __iter__(self):
        cycle=-1;order=None;batches=len(self.rows)//32
        for step in range(self.epoch*self.steps,(self.epoch+1)*self.steps):
            rng=random.Random(20261016+step)
            if self.aggressive:
                hard=rng.sample(self.h,12);used=set(hard)
                positive=rng.sample([i for i in self.a if i not in used],10)
                indices=hard+positive+rng.sample(self.b,10)
                rng.shuffle(indices)
            else:
                current=step//batches
                if current!=cycle:
                    cycle=current
                    order=torch.randperm(len(self.rows),generator=torch.Generator().manual_seed(20261016+cycle)).tolist()
                offset=(step%batches)*32;indices=order[offset:offset+32]
            assert len(indices)==32 and len(set(indices))==32
            yield [(index,20261017+step*32+position) for position,index in enumerate(indices)]


def road_training_loader(bank,aggressive,steps=214,workers=2):
    dataset=DeterministicRoadDataset()
    sampler=RoadBatchSampler(dataset.rows,bank,aggressive,steps)
    return DataLoader(dataset,batch_sampler=sampler,num_workers=workers,pin_memory=torch.cuda.is_available(),
        persistent_workers=workers>0,worker_init_fn=seed_worker,
        generator=torch.Generator().manual_seed(20261016))


@torch.inference_mode()
def mine_hard_pool(weights,attributes,output):
    manifest=json.loads((SEG_ROOT/'manifest.json').read_text())
    day=[r for r in manifest['train'] if attributes.get(r['id'],{}).get('timeofday')=='daytime']
    a=sorted(r['id'] for r in day if r['pixels'][2]>=512)
    b=sorted(r['id'] for r in day if r['pixels'][2]<512)
    assert len(day)==4785 and len(a)==2554 and len(b)==2231
    assert not (set(a+b)&{r['id'] for r in manifest['val']})
    model=load_model(weights).cuda().float().eval().fuse(verbose=False)
    rankings=[]
    for number,batch in enumerate(seg_loader('train',batch=32,workers=0,ids=a)):
        mask=batch['mask'].cuda()
        pred=F.interpolate(model.predict_semantic(batch['img'].cuda().float()/255),mask.shape[-2:],mode='bilinear',align_corners=False).argmax(1)
        inside=interior_regions(mask)[:,1]
        counts=inside.flatten(1).sum(1).cpu().tolist()
        errors=((pred==1)&inside).flatten(1).sum(1).cpu().tolist()
        for identity,n,e in zip(batch['id'],counts,errors):
            rankings.append({'id':identity,'interior_pixels':n,'alt_as_direct_pixels':e,'score':e/n if n else -1.})
        if number%16==0:print(f'HARD_MINING {len(rankings)}/{len(a)}',flush=True)
    rankings.sort(key=lambda r:(-r['score'],r['id']))
    result={'source_sha256':hashlib.sha256(weights.read_bytes()).hexdigest(),
        'pool_A':a,'pool_B':b,'pool_H':[r['id'] for r in rankings[:math.ceil(len(a)*.25)]],
        'ranking':rankings,'training_split_only':True,'batch_composition':[12,10,10],
        'score':'C0 alternative-to-direct rate in r4 interior, fallback r2; empty interior=-1',
        'mixed_sampler':'Uniform seeded shuffled cycles, drop final <32 tail per cycle; seed=20261016+cycle',
        'augmentation':'Existing flip/brightness probabilities and ranges, independent per-sample seed for replay'}
    write_json(output,result)
    return result
