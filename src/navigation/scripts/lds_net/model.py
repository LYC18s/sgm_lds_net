"""Small additive model using the installed official YOLO26 Detect and E2ELoss.

This is a derived architecture, not the unmodified official YOLO26n checkpoint.
All feature blocks use ordinary convolutions and ReLU; only output assembly in
the official detection implementation concatenates tensors.
"""
from __future__ import annotations

import copy
import torch
from torch import nn
import torch.nn.functional as F
from ultralytics.nn.modules import Conv, Detect
from ultralytics.nn.tasks import DetectionModel
from ultralytics.utils import DEFAULT_CFG_DICT, IterableSimpleNamespace
from ultralytics.utils.torch_utils import initialize_weights, fuse_conv_and_bn
from .road_loss import boundary_ce_loss
from .aux_boundary import AuxiliaryBoundaryHead,balanced_boundary_bce
from .discrimination import discrimination_loss


def conv(c1, c2, k=3, s=1, act=True):
    return Conv(c1, c2, k, s, act=nn.ReLU(inplace=True) if act else False)


class Add(nn.Module):
    def forward(self, a, b):
        return a + b


class Residual(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.reduce = conv(channels, channels // 2, 1)
        self.expand = conv(channels // 2, channels, 3, act=False)
        self.add = Add()
        self.act = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.act(self.add(x, self.expand(self.reduce(x))))


class RoadHead(nn.Module):
    def __init__(self, width=16, classes=3, fuse_p2=True):
        super().__init__()
        self.fuse_p2 = fuse_p2
        self.p3 = conv(64, width, 1)
        self.p2 = conv(32, width, 1) if fuse_p2 else None
        self.add = Add()
        self.refine = conv(width, width, 3)
        self.output = nn.Conv2d(width, classes, 1)

    def add_context(self):
        if getattr(self, 'context', None) is None:
            self.context = nn.Conv2d(64, 16, 1, bias=False).to(self.output.weight)
            nn.init.zeros_(self.context.weight)
        return self

    def forward(self, p2, p3, f4=None):
        x = self.p3(p3)
        if getattr(self, 'context', None) is not None:
            if f4 is None:
                raise ValueError('Context road head requires the shared F4 feature')
            x = self.add(x, F.interpolate(self.context(f4), size=x.shape[-2:], mode='nearest'))
        if getattr(self,'semantic_refine',None) is not None:
            x=self.add(x,self.semantic_refine(x))
        if self.fuse_p2:
            x = self.add(F.interpolate(x, scale_factor=2., mode='nearest'), self.p2(p2))
        return self.output(self.refine(x))

    def add_semantic_refinement(self):
        if getattr(self,'semantic_refine',None) is None:
            self.semantic_refine=nn.Sequential(conv(16,32,1),conv(32,32,3),conv(32,32,3),
                nn.Conv2d(32,16,1,bias=False)).to(self.output.weight)
            initialize_weights(self.semantic_refine)
            nn.init.zeros_(self.semantic_refine[-1].weight)
            self.semantic_refine.train(self.training)
        return self


def segmentation_loss(logits, mask):
    """CE + foreground Dice; padding/unknown 255 contributes to neither term."""
    logits = F.interpolate(logits.float(), size=mask.shape[-2:], mode='bilinear', align_corners=False)
    valid = mask != 255
    if not valid.any():
        zero = logits.sum() * 0
        return zero, zero
    # Flatten avoids the platform-specific CUDA nll_loss2d path.
    ce = F.cross_entropy(logits.permute(0, 2, 3, 1)[valid], mask[valid].long())
    probabilities = logits.softmax(1)
    terms = []
    for category in (1, 2):
        target = (mask == category).float()
        pred = probabilities[:, category] * valid
        terms.append(1 - (2 * (pred * target).sum() + 1) / (pred.sum() + target.sum() + 1))
    return ce, torch.stack(terms).mean()


class LDSNetModel(DetectionModel):
    def __init__(self, nc=10, names=None):
        nn.Module.__init__(self)
        # Stable indices: P2=2, P3=4, P4=6, P5=8, projections=9/10, fusion=11.
        layers = [conv(3, 16, 3, 2), conv(16, 32, 3, 2), Residual(32),
                  conv(32, 64, 3, 2), Residual(64), conv(64, 96, 3, 2), Residual(96),
                  conv(96, 128, 3, 2), Residual(128), conv(128, 64, 1),
                  conv(96, 64, 1), conv(64, 64, 3)]
        head = Detect(nc=nc, reg_max=1, end2end=True, ch=(64, 64))
        # Keep the official head API, assignment and loss, with ordinary thin cls convs.
        head.cv3 = nn.ModuleList(nn.Sequential(conv(c, 24), conv(24, 24), nn.Conv2d(24, nc, 1))
                                 for c in (64, 64))
        head.one2one_cv3 = copy.deepcopy(head.cv3)
        layers.append(head)
        self.model = nn.Sequential(*layers)
        for i, module in enumerate(self.model):
            module.i, module.f = i, -1
            module.type = type(module).__name__
            module.np = sum(p.numel() for p in module.parameters())
        self.neck_add = Add()
        self.seg_head = None
        self.seg_weight = 0.5
        self.save = [2, 4, 6, 8]
        self.stride = head.stride = torch.tensor([16., 32.])
        self.inplace, self.nc = True, nc
        self.names = names or {i: str(i) for i in range(nc)}
        self.yaml = {'nc': nc, 'channels': 3, 'architecture': 'lds_net-p45-add',
                     'backbone': [[-1, 1, 'Conv', [16, 3, 2]]],
                     'head': [[[6, 8], 1, 'Detect', [nc]]],
                     'end2end': True, 'reg_max': 1}
        self.args = IterableSimpleNamespace(**{**DEFAULT_CFG_DICT, 'task': 'detect'})
        self.criterion = None
        initialize_weights(self)
        # Official Detect creates ReLU-incompatible default activations in box branches.
        for module in self.modules():
            if isinstance(module, Conv) and isinstance(module.act, nn.SiLU):
                module.act = nn.ReLU(inplace=True)
        head.bias_init()
        self.end2end = True

    def features(self, x, shallow=False):
        x = self.model[0](x)
        p2 = self.model[2](self.model[1](x))
        p3 = self.model[4](self.model[3](p2))
        if shallow:
            return p2, p3
        p4 = self.model[6](self.model[5](p3))
        p5 = self.model[8](self.model[7](p4))
        f5 = self.model[9](p5)
        fused = self.neck_add(self.model[10](p4), F.interpolate(f5, scale_factor=2., mode='nearest'))
        if getattr(self, 'det_p3_project', None) is not None:
            fused = self.neck_add(fused, self.det_p3_down(self.det_p3_project(p3)))
        f4 = self.model[11](fused)
        return p2, p3, f4, f5

    def _predict_once(self, x, profile=False, embed=None):
        *_, p4, p5 = self.features(x)
        return self.model[-1]([p4, p5])

    def predict_semantic(self, x):
        if self.seg_head is None:
            raise RuntimeError('This checkpoint has no road segmentation branch')
        if getattr(self.seg_head, 'context', None) is not None:
            p2, p3, f4, _ = self.features(x)
            return self.seg_head(p2, p3, f4)
        return self.seg_head(*self.features(x, shallow=True))

    def predict_both(self, x):
        p2, p3, p4, p5 = self.features(x)
        return self.model[-1]([p4, p5]), self.seg_head(p2, p3, p4) if self.seg_head else None

    def add_auxiliary_boundary(self):
        if getattr(self,'boundary_head',None) is None:
            self.boundary_head=AuxiliaryBoundaryHead().to(next(self.parameters()))
            initialize_weights(self.boundary_head)
            self.boundary_head.train(self.training)
        return self

    def predict_semantic_with_boundary(self,x):
        p2,p3,f4,_=self.features(x)
        return self.seg_head(p2,p3,f4),self.boundary_head(p2)

    def add_light_fusion(self, road_context=False, detection_detail=False):
        if road_context:
            self.add_segmentation()
            self.seg_head.add_context()
        if detection_detail and getattr(self, 'det_p3_project', None) is None:
            self.det_p3_project = conv(64, 32, 1).to(next(self.parameters()))
            self.det_p3_down = nn.Conv2d(32, 64, 3, 2, 1, bias=False).to(next(self.parameters()))
            initialize_weights(self.det_p3_project)
            nn.init.zeros_(self.det_p3_down.weight)
        self.train(self.training)
        return self

    def add_segmentation(self, fuse_p2=True):
        if self.seg_head is None:
            self.seg_head = RoadHead(fuse_p2=fuse_p2).to(next(self.parameters()).device)
            initialize_weights(self.seg_head)
            self.seg_head.train(self.training)
        return self

    def loss(self, batch, preds=None):
        if self.criterion is None:
            self.criterion = self.init_criterion()
        preds = self.predict(batch['img']) if preds is None else preds
        det, items = self.criterion(preds, batch)
        total = det.sum()
        if self.seg_head is not None:
            ce = dice = band_ce = aux_bce = da_loss = total.detach() * 0
            dice_weight=getattr(self,'seg_dice_weight',1.)
            boundary_weight=getattr(self,'seg_boundary_weight',0.)
            aux_weight=getattr(self,'seg_aux_weight',0.)
            da_weight=getattr(self,'seg_da_weight',0.)
            if 'seg_img' in batch:
                if getattr(self,'seg_aux_enabled',False):
                    logits,aux_logits=self.predict_semantic_with_boundary(batch['seg_img'])
                    aux_bce=balanced_boundary_bce(aux_logits,batch['seg_mask'])
                else:
                    logits=self.predict_semantic(batch['seg_img'])
                ce, dice = segmentation_loss(logits, batch['seg_mask'])
                if boundary_weight:
                    band_ce=boundary_ce_loss(logits,batch['seg_mask'],getattr(self,'seg_boundary_radius',2))
                if getattr(self,'seg_da_enabled',False):
                    da_loss=discrimination_loss(logits,batch['seg_mask'],getattr(self,'seg_da_seed',20261017))
                road=ce+dice_weight*dice+boundary_weight*band_ce+aux_weight*aux_bce+da_weight*da_loss
                total = total + self.seg_weight * road * batch['img'].shape[0]
            items = {**items, 'road_ce': ce.detach(), 'road_dice': dice.detach()}
            if hasattr(self,'seg_boundary_weight'):
                items['road_boundary_ce']=band_ce.detach()
            if hasattr(self,'seg_aux_enabled'):
                items['road_aux_bce']=aux_bce.detach()
            if hasattr(self,'seg_da_enabled'):
                items['road_da']=da_loss.detach()
            if getattr(self, 'record_task_losses', False):
                road=ce+dice_weight*dice+boundary_weight*band_ce+aux_weight*aux_bce+da_weight*da_loss
                items.update({'det_raw': det.sum().detach() / batch['img'].shape[0],
                              'road_raw': road.detach(),
                              'road_weighted': self.seg_weight * road.detach()})
        return total, items

    def fuse(self, verbose=True, imgsz=320):
        # Include the segmentation head, which intentionally sits outside model[-1].
        fold_batchnorm(self)
        self.model[-1].fuse()
        if verbose:
            self.info(imgsz=imgsz)
        return self


def fold_batchnorm(model):
    """Fold only BN; keep both detection branches for QAT training."""
    for module in model.modules():
        if isinstance(module, Conv) and hasattr(module, 'bn'):
            module.conv = fuse_conv_and_bn(module.conv, module.bn)
            delattr(module, 'bn')
            module.forward = module.forward_fuse
    return model


class RawExport(nn.Module):
    """One shared feature pass; separate PL outputs, no sigmoid/top-k/Concat."""
    def __init__(self, model):
        super().__init__()
        self.network = model

    def forward(self, x):
        p2, p3, p4, p5 = self.network.features(x)
        head = self.network.model[-1]
        boxes = head.one2one_cv2
        classes = head.one2one_cv3
        output = (boxes[0](p4), classes[0](p4), boxes[1](p5), classes[1](p5))
        if self.network.seg_head is not None:
            output += (self.network.seg_head(p2, p3, p4),)
        return output
