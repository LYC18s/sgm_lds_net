"""Integer-only convolution/Add/requantization and real OIHW low-bit packing.

CPU torch integer matmul is used for convolution; no float convolution is hidden
in this reference. Floating point is only used when constructing parameters and
when explicitly dequantizing the five terminal tensors for PS evaluation.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
from .data import write_json


def multiplier(ratio):
    fraction, exponent = math.frexp(float(ratio))
    m, shift = round(fraction * (1 << 31)), 31-exponent
    if m == 1 << 31:
        m //= 2
        shift -= 1
    if not 0 <= shift <= 62:
        raise OverflowError(f'Unsupported requantization ratio {ratio}: shift {shift}')
    return m, shift


def round_shift_even(x, shift):
    """Symmetric RNE, including negative halfway values; no signed-shift bias."""
    if shift == 0:
        return x
    negative = x < 0
    a = x.abs()
    whole = a >> shift
    remainder = a - (whole << shift)
    half = 1 << (shift-1)
    whole = whole + ((remainder > half) | ((remainder == half) & ((whole & 1) != 0))).long()
    return torch.where(negative, -whole, whole)


def tensor_multiplier(ratio):
    """Same frexp/RNE parameters as multiplier(), without a CUDA->CPU sync."""
    fraction, exponent = torch.frexp(ratio.double())
    m = (fraction*(1<<31)).round().long()
    carry = m == (1<<31)
    return torch.where(carry,m//2,m), 31-exponent.long()-carry.long()


def tensor_round_shift_even(x, shift):
    """Broadcast per-channel shifts, including zero; no Python channel loops."""
    a=x.abs()
    whole=a >> shift
    remainder=a-(whole << shift)
    half=torch.ones_like(shift) << (shift-1).clamp_min(0)
    increment=(shift>0)&((remainder>half)|((remainder==half)&((whole&1)!=0)))
    whole=whole+increment.long()
    return torch.where(x<0,-whole,whole)


def pack_weights(values, bits):
    q = np.asarray(values, dtype=np.int8).reshape(-1)
    lo, hi = -2**(bits-1), 2**(bits-1)-1
    if q.size and (int(q.min()) < lo or int(q.max()) > hi):
        raise ValueError('Out-of-range packed weight')
    if bits == 8:
        return q.tobytes()
    if bits not in (2,4):
        raise ValueError('Only signed INT8, INT4 and INT2 weights are supported')
    per_byte=8//bits
    codes=q.view(np.uint8)&(2**bits-1)
    codes=np.pad(codes,(0,(-len(codes))%per_byte)).reshape(-1,per_byte)
    result=np.zeros(len(codes),np.uint8)
    for i in range(per_byte):result |= codes[:,i] << (i*bits)
    return result.tobytes()


def unpack_weights(data, bits, shape):
    if bits == 8:
        q = np.frombuffer(data, dtype=np.int8).copy()
    elif bits in (2,4):
        b = np.frombuffer(data, dtype=np.uint8)
        q = np.stack([(b>>(i*bits))&(2**bits-1) for i in range(8//bits)],axis=1).reshape(-1).astype(np.int8)
        q[q >= 2**(bits-1)] -= 2**bits
    else:raise ValueError(bits)
    return q[:math.prod(shape)].reshape(shape)


def export_integer(graph, directory):
    from .deploy_quant import bit_table
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    spec = {'format': 'lds_net.integer.v1', 'stage': graph.stage,
            'input': {'shape': [1,3,320,320], 'bits': 8, 'scale': 1/255, 'zero_point': -128},
            'outputs': graph.outputs, 'nodes': bit_table(graph),
            'packing': 'OIHW, least-significant code first in each byte, signed two\u2019s complement',
            'rounding': 'nearest ties to even; saturate; no wraparound',
            'weights_file': 'weights.bin', 'auxiliary_file': 'parameters.npz'}
    arrays = {}
    packed = bytearray()
    for row in spec['nodes']:
        name = row['id']
        out = graph.qspec(name)
        if row['op'] == 'conv':
            op = graph.ops[name]
            conv = op.conv
            ws = op.weight_scale.detach().cpu().numpy().astype(np.float64)
            iq = graph.qspec(row['inputs'][0])
            sin = float(iq.scale)
            q = (op.effective_weight().detach().cpu()/op.weight_scale.detach().cpu()[:,None,None,None]).round().clamp(
                -2**(op.bits-1),2**(op.bits-1)-1).to(torch.int8).numpy()
            b = np.zeros(conv.out_channels) if conv.bias is None else conv.bias.detach().cpu().numpy().astype(np.float64)
            bf = (torch.from_numpy(b).float()/(iq.scale.detach().cpu()*op.weight_scale.detach().cpu())).round().double().numpy()
            if np.any(np.abs(bf) > 2147483647):
                raise OverflowError(f'{name}: INT32 bias overflow')
            bi = bf.astype(np.int32)
            lo, hi = iq.limits
            maximum = max(abs(lo-iq.zero), abs(hi-iq.zero))
            bound = np.abs(q.astype(np.int64)).reshape(len(q), -1).sum(1)*maximum + np.abs(bi.astype(np.int64))
            if bound.max() > 2147483647:
                raise OverflowError(f'{name}: INT32 accumulator bound {bound.max()}')
            ms = [multiplier(sin*s/float(out.scale)) for s in ws]
            if max(int(bound[c])*abs(ms[c][0]) for c in range(len(ws))) > 9223372036854775807:
                raise OverflowError(f'{name}: INT64 requantization product overflow')
            payload = pack_weights(q, op.bits)
            row.update(weight_shape=list(q.shape), weight_offset=len(packed), weight_bytes=len(payload),
                       stride=list(conv.stride), padding=list(conv.padding), dilation=list(conv.dilation),
                       groups=conv.groups, accumulator_abs_bound=int(bound.max()),
                       multiplier=[m for m,s in ms], shift=[s for m,s in ms],
                       input_scale=sin, input_zero_point=iq.zero)
            packed.extend(payload)
            arrays[name+'__bias'] = bi
            arrays[name+'__weight_scale'] = ws.astype(np.float32)
        elif row['op'] == 'add':
            ratios = [float(graph.qspec(k).scale)/float(out.scale) for k in row['inputs']]
            ms = [multiplier(v) for v in ratios]
            shift = min(s for m,s in ms)
            mult = [round(v * (1 << shift)) for v in ratios]
            bounds = [max(abs(v) for v in graph.qspec(k).limits) for k in row['inputs']]
            if sum(b*abs(m) for b,m in zip(bounds,mult)) > 9223372036854775807:
                raise OverflowError(f'{name}: Add INT64 alignment overflow')
            row.update(multiplier=mult, shift=shift,
                       input_zero_points=[graph.qspec(k).zero for k in row['inputs']])
    (directory/'weights.bin').write_bytes(packed)
    np.savez(directory/'parameters.npz', **arrays)
    spec['storage'] = {'weight_payload_bytes': len(packed),
        'bias_payload_bytes': sum(v.nbytes for k,v in arrays.items() if k.endswith('__bias')),
        'weight_scale_payload_bytes': sum(v.nbytes for k,v in arrays.items() if k.endswith('__weight_scale')),
        'parameters_npz_container_bytes': (directory/'parameters.npz').stat().st_size,
        'note': 'JSON metadata/NPZ headers are containers, not additional low-bit weights.'}
    write_json(directory/'graph.json', spec)
    write_json(directory/'memory.json', graph.report())
    return spec


class IntegerReference:
    def __init__(self, directory):
        directory = Path(directory)
        self.spec = json.loads((directory/'graph.json').read_text(encoding='utf-8'))
        payload = (directory/'weights.bin').read_bytes()
        with np.load(directory/'parameters.npz') as archive:
            self.arrays = {k:archive[k].copy() for k in archive.files}
        self.weights = {}
        for r in self.spec['nodes']:
            if r['op'] == 'conv':
                p = payload[r['weight_offset']:r['weight_offset']+r['weight_bytes']]
                self.weights[r['id']] = torch.from_numpy(unpack_weights(p, r['weight_bits'], r['weight_shape']).copy()).int()

    @staticmethod
    def conv(x, w, row):
        if row['groups'] != 1 or row['dilation'] != [1,1]:
            raise ValueError('Reference currently targets the verified ordinary R2 convolutions')
        centered = x.int()-row['input_zero_point']
        ph,pw = row['padding']; sh,sw = row['stride']
        centered = F.pad(centered, (pw,pw,ph,ph), value=0)
        kh,kw = w.shape[-2:]
        patches = centered.unfold(2,kh,sh).unfold(3,kw,sw)
        n,c,h,v,_,_ = patches.shape
        matrix = patches.permute(1,4,5,0,2,3).reshape(c*kh*kw, n*h*v).contiguous()
        # Bounds were checked at export, including bias, for every output channel.
        acc = w.reshape(len(w),-1) @ matrix
        return acc.reshape(len(w),n,h,v).permute(1,0,2,3).contiguous()

    def __call__(self, images, return_nodes=False):
        if isinstance(images, np.ndarray):
            images = torch.from_numpy(images)
        if images.dtype != torch.int8:
            raise TypeError('Integer core input must be p-128 encoded INT8')
        values = {'input': images.cpu()}
        remaining = {}
        for row in self.spec['nodes']:
            for k in row['inputs']:
                remaining[k] = remaining.get(k,0)+1
        for row in self.spec['nodes']:
            name = row['id']; args = [values[k] for k in row['inputs']]
            if row['op'] == 'upsample':
                out = args[0].repeat_interleave(2,-2).repeat_interleave(2,-1)
            elif row['op'] == 'conv':
                out = self.conv(args[0], self.weights[name], row).long()
                out += torch.from_numpy(self.arrays[name+'__bias'].copy()).long()[None,:,None,None]
                m = torch.tensor(row['multiplier'],dtype=torch.int64)[None,:,None,None]
                shifts = row['shift']
                out = torch.stack([round_shift_even(out[:,c]*m[:,c], shifts[c]) for c in range(out.shape[1])],1)
            else:
                a,b = [v.long()-z for v,z in zip(args,row['input_zero_points'])]
                out = round_shift_even(a*row['multiplier'][0]+b*row['multiplier'][1], row['shift'])
            if row['op'] != 'upsample':
                bits = row['output_bits']
                lo,hi = (-2**(bits-1),2**(bits-1)-1) if row['output_signed'] else (0,2**bits-1)
                out = out.clamp(lo,hi).to(torch.int8 if row['output_signed'] else torch.uint8)
            values[name] = out
            if not return_nodes:
                for k in row['inputs']:
                    remaining[k] -= 1
                    if remaining[k] == 0 and k not in self.spec['outputs']:
                        values.pop(k,None)
        return values if return_nodes else tuple(values[k] for k in self.spec['outputs'])

    def dequantize(self, outputs):
        by_id = {r['id']:r for r in self.spec['nodes']}
        return tuple(x.float()*by_id[k]['output_scale'] for x,k in zip(outputs,self.spec['outputs']))
