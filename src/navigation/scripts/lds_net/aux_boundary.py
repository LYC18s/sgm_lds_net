"""Two-class boundary supervision, used only during joint training."""
import copy

import torch
from torch import nn
import torch.nn.functional as F


class AuxiliaryBoundaryHead(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers=nn.Sequential(nn.Conv2d(32,8,3,padding=1,bias=False),
            nn.BatchNorm2d(8),nn.ReLU(inplace=True),nn.Conv2d(8,2,1))

    def forward(self,p2):
        return self.layers(p2)


def auxiliary_targets(mask,stride=4,ignore_radius=2):
    """Generate thin class boundaries BEFORE max-pooling to the P2 grid."""
    if mask.shape[-2]%stride or mask.shape[-1]%stride:
        raise ValueError('Mask size must be divisible by the auxiliary stride')
    valid=mask!=255
    safe=valid[:,None]
    if ignore_radius:
        safe=safe & (F.max_pool2d((~valid)[:,None].float(),2*ignore_radius+1,1,ignore_radius)<.5)
    targets=[]
    for category in (1,2):
        region=(mask==category)[:,None].float()
        # Same inner class-edge convention as the boundary evaluation.
        edge=(region+F.max_pool2d(-region,3,1,1)>.5) & safe
        targets.append(F.max_pool2d(edge.float(),stride,stride))
    low_valid=F.max_pool2d((~safe).float(),stride,stride)<.5
    target=torch.cat(targets,1)
    return target,low_valid.expand_as(target)


def balanced_boundary_bce(logits,mask):
    target,valid=auxiliary_targets(mask)
    if logits.shape!=target.shape:
        raise ValueError(f'Auxiliary logits/target shape mismatch: {logits.shape} vs {target.shape}')
    pixel=F.binary_cross_entropy_with_logits(logits.float(),target,reduction='none')
    terms=[]
    for category in range(2):
        positive=valid[:,category] & (target[:,category]>.5)
        negative=valid[:,category] & ~positive
        parts=[]
        for selected in (positive,negative):
            if selected.any():parts.append(pixel[:,category][selected].mean())
        if parts:terms.append(torch.stack(parts).mean())
    return torch.stack(terms).mean() if terms else logits.float().sum()*0


def auxiliary_weight(epoch,batch,batches,enabled=True):
    if batches<=0:raise ValueError('Empty training loader')
    return .1*min(max((epoch*batches+batch)/(2*batches),0.),1.) if enabled else 0.


def strip_auxiliary(model,inplace=False):
    result=model if inplace else copy.deepcopy(model)
    if hasattr(result,'boundary_head'):delattr(result,'boundary_head')
    result.seg_aux_enabled=False
    result.seg_aux_weight=0.
    return result
