"""Deployment-aligned fake quantization, histogram calibration and bit policies."""
from __future__ import annotations
import copy
import math
import re
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from .deploy_graph import ConvOp, AddOp


class TensorQuant(nn.Module):
    def __init__(self, bits=8, unsigned=False, fixed_input=False):
        super().__init__()
        self.bits, self.unsigned, self.fixed_input = bits, unsigned, fixed_input
        self.register_buffer('scale', torch.tensor(1/255 if fixed_input else 1., dtype=torch.float32))
        self.zero = -128 if fixed_input else 0
        self.enabled = True
        self.observe = False
        self.collect = None
        self.histogram = None
        self.maximum = 0.
        self.calibration = {}

    @property
    def limits(self):
        return (0, 2**self.bits-1) if self.unsigned else (-2**(self.bits-1), 2**(self.bits-1)-1)

    def forward(self, x):
        if self.collect:
            v = x.detach().float().abs() if not self.unsigned else x.detach().float().clamp_min(0)
            if self.collect == 'range':
                self.maximum = max(self.maximum, float(v.max()))
            elif self.collect == 'histogram':
                h = torch.histc(v, bins=2048, min=0., max=max(self.maximum, 1e-8))
                self.histogram += h.cpu().double().numpy()
        if self.observe and not self.fixed_input:
            high = float(x.detach().abs().max())
            new = max(high / self.limits[1], 1e-8)
            self.scale.mul_(.99).add_(new*.01)
        if not self.enabled:
            return x
        lo, hi = self.limits
        return torch.fake_quantize_per_tensor_affine(x.float(), self.scale, torch.tensor(
            self.zero, device=x.device, dtype=torch.int32), lo, hi)

    def choose_histogram(self):
        hist = self.histogram
        centers = (np.arange(2048)+.5) * max(self.maximum, 1e-8)/2048
        cumulative = np.cumsum(hist)
        if not cumulative[-1]:
            raise ValueError('Empty calibration histogram')
        candidates = []
        for p in (.999, .9995, .9999, 1.):
            idx = min(int(np.searchsorted(cumulative, p*cumulative[-1])), 2047)
            clip = max(self.maximum, 1e-8) * (idx+1)/2048
            s = max(clip/self.limits[1], 1e-8)
            # Signed negative bins have one extra representable code. Absolute
            # histogram uses the conservative positive endpoint for both signs.
            dequant = np.clip(np.rint(centers/s), 0, self.limits[1])*s
            mse = float(np.dot(hist, (centers-dequant)**2)/cumulative[-1])
            candidates.append({'percentile': p, 'scale': s, 'mse': mse})
        best = min(candidates, key=lambda r: (r['mse'], -r['percentile']))
        self.scale.fill_(best['scale'])
        self.calibration = {'bins': 2048, 'maximum': self.maximum, 'candidates': candidates, 'selected': best}
        self.histogram = None
        self.collect = None


def stage_bits(stage):
    match=re.fullmatch(r'W(2|4|8)A(4|6|8)',stage)
    if match is None:raise ValueError(stage)
    return tuple(map(int,match.groups()))


class QConv(ConvOp):
    def __init__(self, original, input_quant, bits=8, activation_bits=8):
        super().__init__(original.conv, original.relu)
        self.bits = bits
        object.__setattr__(self, 'input_quant', input_quant)
        self.output_quant = TensorQuant(activation_bits, unsigned=self.relu)
        self.register_buffer('weight_scale', torch.ones(self.conv.out_channels))
        self.update_weights = True
        self.enabled = True
        self.initialize_weight_scale()

    def effective_weight(self):
        mask = getattr(self, 'weight_mask', None)
        return self.conv.weight if mask is None else self.conv.weight*mask

    def initialize_weight_scale(self):
        with torch.no_grad():
            peak=self.effective_weight().detach().abs().flatten(1).amax(1).clamp_min(1e-8)
            if self.bits==2 and hasattr(self,'w2_scale_ratio'):
                self.weight_scale.copy_(peak*self.w2_scale_ratio)
            else:
                self.weight_scale.copy_(peak/(2**(self.bits-1)-1))

    def initialize_two_bit_ratio(self):
        """Per-channel MSE initialization for all four signed codes [-2,1]."""
        with torch.no_grad():
            w=self.conv.weight.detach().float().flatten(1)
            peak=w.abs().amax(1).clamp_min(1e-8)
            ratios=torch.tensor([.2,.25,.3,.35,.4,.45,.5,.6,.7,.8,1.],device=w.device)
            errors=[]
            for ratio in ratios:
                s=peak*ratio
                dq=(w/s[:,None]).round().clamp(-2,1)*s[:,None]
                errors.append((dq-w).square().mean(1))
            selected=ratios[torch.stack(errors).argmin(0)]
            if hasattr(self,'w2_scale_ratio'):
                self.w2_scale_ratio.copy_(selected)
            else:
                self.register_buffer('w2_scale_ratio',selected.clone())
    def forward(self, x):
        if not self.enabled:
            if not hasattr(self, 'weight_mask'):
                return self.output_quant(super().forward(x))
            value = F.conv2d(x, self.effective_weight(), self.conv.bias, self.conv.stride,
                            self.conv.padding, self.conv.dilation, self.conv.groups)
            return self.output_quant(value.relu() if self.relu else value)
        if self.update_weights:
            self.initialize_weight_scale()
        lo, hi = -2**(self.bits-1), 2**(self.bits-1)-1
        # Accumulate integer-valued codes in FP32 for differentiable CUDA QAT.
        # Requantization itself uses the exact INT64 M/R path, so halfway ties do
        # not propagate several LSBs through later layers.
        weight_scale = self.weight_scale.clone()
        ws = weight_scale[:,None,None,None]
        codes = self.effective_weight().float()/ws
        w = codes+(codes.round().clamp(lo,hi)-codes).detach()
        sin = self.input_quant.scale.clone()
        xc = x.float()/sin
        xcodes = xc+(xc.round()-xc).detach()
        b = self.conv.bias
        if b is not None:
            bs = sin * weight_scale
            bc = b.float()/bs
            b = bc+(bc.round().clamp(-2147483648, 2147483647)-bc).detach()
        acc = F.conv2d(xcodes, w, b, self.conv.stride, self.conv.padding,
                         self.conv.dilation, self.conv.groups)
        value = acc*(sin*weight_scale)[None,:,None,None]
        if self.relu:
            value = value.relu()
        # Observe before constructing the integer multipliers.
        if self.output_quant.observe or self.output_quant.collect:
            self.output_quant(value)
        from .integer_reference import tensor_multiplier,tensor_round_shift_even
        with torch.no_grad():
            ratios=sin.double()*self.weight_scale.double()/self.output_quant.scale.double()
            mult,shift=tensor_multiplier(ratios)
            product=acc.detach().round().long()*mult[None,:,None,None]
            exact=tensor_round_shift_even(product,shift[None,:,None,None])
            qlo,qhi=self.output_quant.limits
            exact=exact.clamp(qlo,qhi).float()*self.output_quant.scale
        return value+(exact-value).detach()


class QAdd(AddOp):
    def __init__(self, original, inputs, bits=8):
        super().__init__(original.relu)
        object.__setattr__(self, 'input_quants', inputs)
        self.output_quant = TensorQuant(bits, unsigned=self.relu)

    def forward(self, a, b):
        value=super().forward(a,b)
        if self.output_quant.observe or self.output_quant.collect:
            self.output_quant(value)
        if not self.output_quant.enabled:
            return value
        from .integer_reference import tensor_multiplier,tensor_round_shift_even
        with torch.no_grad():
            scales=torch.stack([q.scale for q in self.input_quants]).double()
            ratios=scales/self.output_quant.scale.double()
            shift=tensor_multiplier(ratios)[1].min()
            ms=torch.ldexp(ratios,shift.int()).round().long()
            codes=[(v/q.scale).round().long() for v,q in zip((a,b),self.input_quants)]
            exact=tensor_round_shift_even(codes[0]*ms[0]+codes[1]*ms[1],shift)
            lo,hi=self.output_quant.limits
            exact=exact.clamp(lo,hi).float()*self.output_quant.scale
        return value+(exact-value).detach()


def prepare_graph(source, stage='W8A8'):
    graph = copy.deepcopy(source).float().eval()
    graph.input_quant = TensorQuant(fixed_input=True)
    # The FX graph uses one module per actual Add invocation, including all
    # three invocations of the reused RoadHead.add in the training architecture.
    for row in graph.nodes:
        if row['op'] == 'upsample':
            continue
        inputs = [graph.qspec(k) for k in row['inputs']]
        old = graph.ops[row['id']]
        graph.ops[row['id']] = QConv(old, inputs[0]) if row['op']=='conv' else QAdd(old, inputs)
    graph.quantized = True
    set_policy(graph, stage)
    for p in graph.parameters():
        p.requires_grad_(True)
    return graph


def set_policy(graph, stage, rescue_convs=(), rescue_adds=()):
    weight_bits,activation_bits=stage_bits(stage)
    for row in graph.nodes:
        if row['op'] == 'upsample':
            continue
        op = graph.ops[row['id']]
        output = row['id'] in graph.outputs
        first = row.get('source') == 'model.0.conv'
        # Preserve 8-bit producer features, not a 4-bit feature upcast at the end.
        terminal_input = any(row['id'] in r['inputs'] for r in graph.nodes if r['id'] in graph.outputs)
        op.output_quant.bits = 8 if output or terminal_input or row['id'] in rescue_adds else activation_bits
        op.output_quant.unsigned = op.relu and not output
        if row['op'] == 'conv':
            old_bits=op.bits
            op.bits = 8 if first or output or row['id'] in rescue_convs else weight_bits
            if op.bits==2 and (old_bits!=2 or not hasattr(op,'w2_scale_ratio')):
                op.initialize_two_bit_ratio()
            op.initialize_weight_scale()
    graph.stage = stage


def observers(graph, enabled, weights=None):
    for m in graph.modules():
        if isinstance(m, TensorQuant):
            m.observe = enabled and not m.fixed_input
        if isinstance(m, QConv):
            m.update_weights = enabled if weights is None else weights


@torch.no_grad()
def calibrate(graph, image_batches, progress=None):
    """image_batches is a callable: repeat exactly the same unaugmented images."""
    qs = [m for m in graph.modules() if isinstance(m, TensorQuant) and not m.fixed_input]
    observers(graph, False)
    graph.input_quant.enabled = False
    for m in graph.modules():
        if isinstance(m, QConv):
            m.enabled = False
    for phase in ('range', 'histogram'):
        for q in qs:
            q.collect = phase
            q.enabled = False
            if phase == 'range':
                q.maximum = 0.
            else:
                q.histogram = np.zeros(2048, np.float64)
        for i, images in enumerate(image_batches()):
            graph(images)
            if progress:
                progress(phase, i+1)
    for q in qs:
        q.choose_histogram()
        q.enabled = True
    graph.input_quant.enabled = True
    for m in graph.modules():
        if isinstance(m, QConv):
            m.enabled = True
            m.initialize_weight_scale()
    return graph


def bit_table(graph):
    result = []
    for r in graph.nodes:
        q = graph.qspec(r['id'])
        item = {**r, 'input_bits': [graph.qspec(k).bits for k in r['inputs']],
                'output_bits': q.bits, 'output_signed': not q.unsigned,
                'output_scale': float(q.scale), 'output_zero_point': q.zero}
        if r['op'] == 'conv':
            m = graph.ops[r['id']]
            item.update(weight_bits=m.bits, weight_scale=m.weight_scale.detach().cpu().tolist())
        result.append(item)
    return result
