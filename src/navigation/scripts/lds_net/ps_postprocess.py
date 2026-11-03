"""Shared floating/integer PS postprocessing. No NMS and no road softmax."""
import torch
import torch.nn.functional as F


def decode_detection(raw, max_det=300):
    boxes, scores = [], []
    for box, logits, stride in ((raw[0],raw[1],16),(raw[2],raw[3],32)):
        b,_,h,w = box.shape
        yy,xx = torch.meshgrid(torch.arange(h,device=box.device),torch.arange(w,device=box.device),indexing='ij')
        center = torch.stack((xx,yy),0).float().reshape(1,2,-1)+.5
        ltrb = box.reshape(b,4,-1)
        xyxy = torch.cat((center-ltrb[:,:2],center+ltrb[:,2:]),1)*stride
        boxes.append(xyxy.transpose(1,2))
        scores.append(logits.flatten(2).transpose(1,2).sigmoid())
    boxes, scores = torch.cat(boxes,1), torch.cat(scores,1)
    k = min(max_det,scores.shape[1])
    # Stable ties: original anchor order, then class order within that selection.
    anchors = scores.amax(-1).argsort(dim=1,descending=True,stable=True)[:,:k]
    selected = scores.gather(1,anchors[:,:,None].expand(-1,-1,scores.shape[-1]))
    flattened = selected.flatten(1)
    order = flattened.argsort(dim=1,descending=True,stable=True)[:,:k]
    classes = order % scores.shape[-1]
    indices = anchors.gather(1,order//scores.shape[-1])
    return torch.cat((boxes.gather(1,indices[:,:,None].expand(-1,-1,4)),
                      flattened.gather(1,order)[:,:,None],classes[:,:,None].float()),-1)


def restore_road(logits, original_shape, geometry):
    """Interpolate logits, remove padding, restore logits, THEN argmax."""
    _,top,left,nh,nw = geometry
    values = F.interpolate(logits.float(),(320,320),mode='bilinear',align_corners=False)
    values = values[:,:,top:top+nh,left:left+nw]
    values = F.interpolate(values,original_shape,mode='bilinear',align_corners=False)
    return values.argmax(1)


def restore_road_int8(codes, original_shape, geometry):
    """Exact bilinear equivalent for shared-scale integer road channels.

    The positive common output scale cancels in argmax. Keep rational bilinear
    numerators as INT64 to give Python/C++ identical labels at exact score ties.
    This uses the same half-pixel coordinates as align_corners=False.
    """
    def resize_numerators(v,shape):
        ih,iw=v.shape[-2:];oh,ow=shape
        y=((torch.arange(oh,dtype=torch.int64)*2+1)*ih-oh).clamp_min(0)
        x=((torch.arange(ow,dtype=torch.int64)*2+1)*iw-ow).clamp_min(0)
        y0=(y//(2*oh)).clamp_max(ih-1);y1=(y0+1).clamp_max(ih-1)
        x0=(x//(2*ow)).clamp_max(iw-1);x1=(x0+1).clamp_max(iw-1)
        fy=(y-y0*2*oh)[None,None,:,None];fx=(x-x0*2*ow)[None,None,None,:]
        a=v[:,:,y0][:,:,:,x0]*(2*ow-fx)+v[:,:,y0][:,:,:,x1]*fx
        b=v[:,:,y1][:,:,:,x0]*(2*ow-fx)+v[:,:,y1][:,:,:,x1]*fx
        return a*(2*oh-fy)+b*fy
    _,top,left,nh,nw=geometry
    canvas=resize_numerators(codes.cpu().long(),(320,320))
    crop=canvas[:,:,top:top+nh,left:left+nw]
    return resize_numerators(crop,original_shape).argmax(1)
