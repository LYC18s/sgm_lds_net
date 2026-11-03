"""Explicit, inspectable single-pass R2 graph. No training head is retained."""
from __future__ import annotations
import copy
import operator
import torch
from torch import nn, fx
import torch.nn.functional as F
from .model import RawExport, fold_batchnorm

OUTPUT_NAMES = ['p4_box_ltrb', 'p4_class_logits', 'p5_box_ltrb',
                'p5_class_logits', 'road_logits_stride4']


class ConvOp(nn.Module):
    def __init__(self, conv, relu=False):
        super().__init__()
        self.conv = conv
        self.relu = relu

    def forward(self, x):
        x = self.conv(x)
        return x.relu() if self.relu else x


class AddOp(nn.Module):
    def __init__(self, relu=False):
        super().__init__()
        self.relu = relu

    def forward(self, a, b):
        x = a + b
        return x.relu() if self.relu else x


class DeployGraph(nn.Module):
    def __init__(self):
        super().__init__()
        self.ops = nn.ModuleDict()
        self.nodes = []
        self.outputs = []
        self.feature_outputs = []
        self.quantized = False

    @classmethod
    def from_model(cls, source):
        source = copy.deepcopy(source).cpu().float().eval()
        source.criterion = None
        fold_batchnorm(source)
        traced = fx.symbolic_trace(RawExport(source))
        result = cls()
        aliases = {}
        # Conv/ReLU and Add/ReLU are fused without inserting any new activation.
        skip = set()
        def key(n):
            while n in aliases:
                n = aliases[n]
            return n
        for n in traced.graph.nodes:
            if n in skip:
                continue
            if n.op == 'placeholder':
                aliases[n.name] = 'input'
                continue
            if n.op == 'output':
                result.outputs = [key(v.name) for v in n.args[0]]
                continue
            users = list(n.users)
            following = users[0] if len(users) == 1 else None
            relu = (following is not None and following.op == 'call_module'
                    and isinstance(traced.get_submodule(following.target), nn.ReLU))
            if relu:
                skip.add(following)
                aliases[following.name] = n.name
            ins = [key(v.name) for v in n.args if isinstance(v, fx.Node)]
            row = {'id': n.name, 'inputs': ins}
            if n.op == 'call_module':
                module = traced.get_submodule(n.target)
                if isinstance(module, nn.Identity):
                    aliases[n.name] = ins[0]
                    continue
                if not isinstance(module, nn.Conv2d):
                    raise TypeError(f'Unsupported deployment module: {n.target}: {module}')
                row.update(op='conv', source=n.target.removeprefix('network.'), relu=relu)
                result.ops[n.name] = ConvOp(copy.deepcopy(module), relu)
            elif n.op == 'call_function' and n.target == operator.add:
                row.update(op='add', relu=relu)
                result.ops[n.name] = AddOp(relu)
            elif n.op == 'call_function' and n.target == F.interpolate:
                if n.kwargs.get('mode') != 'nearest':
                    raise ValueError('Only nearest interpolation belongs in the core')
                # R2's size= is derived from x.shape; handle that static shape below.
                row.update(op='upsample', factor=2)
            elif n.op == 'call_function' and n.target in (getattr, operator.getitem):
                # These are only static shape expressions for nearest upsampling.
                continue
            else:
                raise TypeError(f'Unsupported deployment node: {n.format_node()}')
            result.nodes.append(row)
        # Locate feature tensors from their source convolution paths.
        names = {r.get('source'): r['id'] for r in result.nodes}
        result.feature_outputs = [
            next(r['id'] for r in result.nodes if r['op'] == 'add' and
                 names['model.2.expand.conv'] in r['inputs']),
            next(r['id'] for r in result.nodes if r['op'] == 'add' and
                 names['model.4.expand.conv'] in r['inputs']),
            names['model.11.conv'], names['model.9.conv']]
        # Shape-expression nodes must never appear as tensor dependencies.
        for r in result.nodes:
            if r['op'] == 'upsample':
                r['inputs'] = r['inputs'][:1]
        result.eval()
        with torch.no_grad():
            result(torch.zeros(1, 3, 320, 320))
        return result

    def required(self, outputs, available=()):
        need = set(outputs)
        for row in reversed(self.nodes):
            if row['id'] in need and row['id'] not in available:
                need.update(row['inputs'])
        return need

    def run(self, values, outputs, observer=None):
        needed = self.required(outputs, values)
        if self.quantized and 'input' in values:
            values['input'] = self.input_quant(values['input'])
        for row in self.nodes:
            name = row['id']
            if name in values or name not in needed:
                continue
            args = [values[k] for k in row['inputs']]
            if row['op'] == 'upsample':
                value = F.interpolate(args[0], scale_factor=2., mode='nearest')
            else:
                value = self.ops[name](*args)
            values[name] = value
            if observer:
                observer(row, value)
        return tuple(values[o] for o in outputs)

    def forward(self, x):
        return self.run({'input': x}, self.outputs)

    def features(self, x):
        return self.run({'input': x}, self.feature_outputs)

    def road(self, features):
        return self.run(dict(zip(self.feature_outputs, features)), self.outputs[-1:])[0]

    def branch(self, x, scale, category):
        index = 0 if scale == 0 else 1
        return self.run({self.feature_outputs[2+index]: x},
                        [self.outputs[scale*2+category]])[0]

    def report(self):
        shapes = {'input': [1, 3, 320, 320]}
        mac = 0
        def collect(r, v):
            nonlocal mac
            shapes[r['id']] = list(v.shape)
            if r['op'] == 'conv':
                c = self.ops[r['id']].conv
                mac += v.numel() * c.in_channels * c.kernel_size[0]*c.kernel_size[1]//c.groups
        with torch.no_grad():
            self.run({'input': torch.zeros(1, 3, 320, 320, device=next(self.parameters()).device)},
                     self.outputs, collect)
        # Allocate an output before releasing last-use inputs: conservative liveness.
        remaining = {k: 0 for k in shapes}
        for r in self.nodes:
            for k in r['inputs']:
                remaining[k] += 1
        for k in self.outputs:
            remaining[k] += 1
        live = {'input'}
        def storage(k):
            import math
            bits = self.qspec(k).bits if self.quantized else 32
            return math.ceil(math.prod(shapes[k])*bits/8)
        peak = storage('input')
        for r in self.nodes:
            live.add(r['id'])
            peak = max(peak, sum(storage(k) for k in live))
            for k in r['inputs']:
                remaining[k] -= 1
                if remaining[k] == 0:
                    live.discard(k)
        return {'conv_count': sum(r['op']=='conv' for r in self.nodes),
                'parameters': sum(p.numel() for p in self.parameters()),
                'conv_MAC': mac, 'conv_GFLOPs': 2*mac/1e9,
                'shapes': shapes, 'output_bytes_int8': 26200,
                'peak_live_tensor_storage_bytes': peak,
                'cache_note': 'Whole-tensor liveness; excludes convolution workspace, line buffers and allocator overhead.'}

    def qspec(self, name):
        if name == 'input':
            return self.input_quant
        row = next(r for r in self.nodes if r['id'] == name)
        if row['op'] == 'upsample':
            return self.qspec(row['inputs'][0])
        return self.ops[name].output_quant
