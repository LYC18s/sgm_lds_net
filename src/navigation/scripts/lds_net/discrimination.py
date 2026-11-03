"""Class-balanced direct/alternative margin loss with independent pixel sampling."""
import torch
import torch.nn.functional as F


def interior_regions(mask):
    regions=torch.stack((mask==1,mask==2),1).float()
    four=-F.max_pool2d(-regions,9,1,4)>.5
    two=-F.max_pool2d(-regions,5,1,2)>.5
    present=four.flatten(2).any(-1)[:,:,None,None]
    return torch.where(present,four,two)


def discrimination_loss(logits,mask,seed=20261017,margin=.5,quota=1024):
    scores=F.interpolate(logits.float(),mask.shape[-2:],mode='bilinear',align_corners=False)
    difference=scores[:,2]-scores[:,1]
    losses=torch.stack((F.softplus(margin+difference),F.softplus(margin-difference)),1).flatten(2)
    regions=interior_regions(mask).flatten(2)
    generator=torch.Generator(device=logits.device).manual_seed(int(seed))
    keys=torch.rand(regions.shape,device=logits.device,generator=generator).masked_fill(~regions,-1.)
    indices=keys.topk(min(quota,keys.shape[-1]),dim=-1,sorted=False).indices
    chosen=regions.gather(2,indices)
    count=chosen.sum(2)
    image_class=(losses.gather(2,indices)*chosen).sum(2)/count.clamp_min(1)
    present=count>0
    per_class=(image_class*present).sum(0)/present.sum(0).clamp_min(1)
    classes=present.any(0)
    return (per_class*classes).sum()/classes.sum().clamp_min(1)


def discrimination_weight(adapt,epoch,batch,batches,enabled):
    if not enabled:return 0.
    if not adapt:return .5
    return .5*min(max((epoch*batches+batch)/max(2*batches-1,1),0.),1.)
