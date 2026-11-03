"""Training-only boundary-band supervision in the original 320 label space."""
import torch
import torch.nn.functional as F


def boundary_band(mask, radius=2):
    """Dilate valid class transitions; exclude ignore pixels and their radius-neighborhood.

    Both sides of each horizontal/vertical transition seed the band, including
    direct/alternative transitions. Image borders and ignore255 are not seeds.
    """
    if radius < 0:
        raise ValueError('Boundary radius must be nonnegative')
    valid=mask!=255
    edge=torch.zeros_like(valid)
    horizontal=valid[:,:,1:] & valid[:,:,:-1] & (mask[:,:,1:]!=mask[:,:,:-1])
    vertical=valid[:,1:,:] & valid[:,:-1,:] & (mask[:,1:,:]!=mask[:,:-1,:])
    edge[:,:,1:] |= horizontal;edge[:,:,:-1] |= horizontal
    edge[:,1:,:] |= vertical;edge[:,:-1,:] |= vertical
    if radius:
        kernel=2*radius+1
        edge=F.max_pool2d(edge[:,None].float(),kernel,1,radius)[:,0]>.5
        safe=F.max_pool2d((~valid)[:,None].float(),kernel,1,radius)[:,0]<.5
        return edge & safe & valid
    return edge & valid


def boundary_ce_loss(logits, mask, radius=2):
    band=boundary_band(mask,radius)
    if not band.any():
        return logits.float().sum()*0
    scores=F.interpolate(logits.float(),mask.shape[-2:],mode='bilinear',align_corners=False)
    return F.cross_entropy(scores.permute(0,2,3,1)[band],mask[band].long())
