"""Portable PyTorch QAT reference. Not a FINN bitstream or a native INT4 kernel.

BN is folded before QAT; ordinary conv inputs and per-output-channel weights are
fake quantized. Each Add uses a shared input quantizer and an output quantizer.
First/output convolutions retain 8 bits in mixed W4A4 mode.
"""
import copy
import torch
from torch import nn
from torch.ao.quantization import FakeQuantize, MovingAverageMinMaxObserver, MovingAveragePerChannelMinMaxObserver
from .model import Add, fold_batchnorm


def fake_quant(bits=8, channel=False):
    kwargs = dict(observer=MovingAveragePerChannelMinMaxObserver if channel else MovingAverageMinMaxObserver,
                  quant_min=-(2 ** (bits - 1)), quant_max=2 ** (bits - 1) - 1,
                  dtype=torch.qint8, qscheme=torch.per_channel_symmetric if channel else torch.per_tensor_symmetric)
    if channel:
        kwargs['ch_axis'] = 0
    return FakeQuantize(**kwargs)


class QuantConv(nn.Conv2d):
    @classmethod
    def from_conv(cls, original, bits):
        layer = cls(original.in_channels, original.out_channels, original.kernel_size, original.stride,
                    original.padding, original.dilation, original.groups, original.bias is not None,
                    original.padding_mode, device=original.weight.device, dtype=original.weight.dtype)
        layer.weight = nn.Parameter(original.weight.detach().clone())
        if original.bias is not None:
            layer.bias = nn.Parameter(original.bias.detach().clone())
        layer.weight_fake_quant = fake_quant(bits, channel=True).to(original.weight.device)
        layer.input_fake_quant = fake_quant(bits).to(original.weight.device)
        layer.bits = bits
        return layer

    def forward(self, x):
        return self._conv_forward(self.input_fake_quant(x), self.weight_fake_quant(self.weight), self.bias)


class QuantAdd(Add):
    def __init__(self, bits):
        super().__init__()
        self.shared = fake_quant(bits)
        self.output = fake_quant(bits)

    def forward(self, a, b):
        # Observe the joint range before quantizing either branch on the same scale.
        observing_enabled = bool(self.shared.observer_enabled[0])
        if not observing_enabled:
            return self.output(self.shared(a) + self.shared(b))
        if observing_enabled:
            with torch.no_grad():
                extrema = torch.stack((torch.minimum(a.min(), b.min()), torch.maximum(a.max(), b.max())))
                self.shared.activation_post_process(extrema.detach())
                scale, zero = self.shared.calculate_qparams()
                self.shared.scale.resize_(scale.shape).copy_(scale)
                self.shared.zero_point.resize_(zero.shape).copy_(zero)
        observing = self.shared.observer_enabled.clone()
        self.shared.disable_observer()
        a, b = self.shared(a), self.shared(b)
        self.shared.observer_enabled.copy_(observing)
        return self.output(a + b)


def prepare_qat(model, bits=8):
    model = copy.deepcopy(model).float().eval()
    model.criterion = None
    fold_batchnorm(model)
    # Snapshot prevents descending into newly created fake-quant modules.
    for name, module in list(model.named_modules()):
        if not name:
            continue
        parent_name, _, attr = name.rpartition('.')
        parent = model.get_submodule(parent_name) if parent_name else model
        if isinstance(module, nn.Conv2d):
            sensitive = name == 'model.0.conv' or not name.endswith('.conv')
            setattr(parent, attr, QuantConv.from_conv(module, 8 if sensitive else bits))
        elif isinstance(module, Add):
            setattr(parent, attr, QuantAdd(bits).to(next(model.parameters()).device))
    model.qat_bits = bits
    return model


def set_observers(model, enabled):
    for module in model.modules():
        if isinstance(module, FakeQuantize):
            module.enable_observer(enabled)


def quantization_report(model):
    return [{'name': name, 'bits': module.bits,
             'weight_scale': module.weight_fake_quant.scale.detach().cpu().tolist(),
             'input_scale': module.input_fake_quant.scale.detach().cpu().tolist()}
            for name, module in model.named_modules() if isinstance(module, QuantConv)]
