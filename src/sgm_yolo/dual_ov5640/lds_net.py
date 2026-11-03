"""LDS-Net board ABI utilities. No CPU convolution or layer scheduler.

The backend is deliberately fail-closed until a matching, verified integrated
LDS-Net HWH is loaded. Importing this module does not allocate CMA or start HW.
"""
import ctypes
import hashlib
import json
from pathlib import Path
import numpy as np

ABI = 0x524C0001
CROP = (80, 0, 480, 480)
CLASS_NAMES = ('person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
               'traffic light', 'fire hydrant', 'stop sign', 'parking meter')


class _Detection(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ('x1', 'y1', 'x2', 'y2', 'score')] + [
        ('category', ctypes.c_int32), ('anchor', ctypes.c_int32)]


class NativeDecoder:
    def __init__(self, graph, library=None):
        if library is None:
            library = Path(__file__).with_name('liblds_net_post.so')
        library = Path(library)
        if not library.is_file():
            raise RuntimeError('Build the LDS-Net C++ decoder with tools/build_lds_net_native.py')
        self.library = ctypes.CDLL(str(library.resolve()))
        self.library.lds_net_post_abi.restype = ctypes.c_uint32
        if self.library.lds_net_post_abi() != ABI:
            raise RuntimeError('LDS-Net native decoder ABI mismatch')
        self.library.lds_net_detect.argtypes = [ctypes.POINTER(ctypes.c_int8), ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_float), ctypes.c_float, ctypes.POINTER(_Detection), ctypes.c_int]
        self.library.lds_net_detect.restype = ctypes.c_int
        self.scales = (ctypes.c_float * 4)(*[p['scale'] for p in graph['outputs'][:4]])

    def __call__(self, raw, threshold=.25):
        source = np.frombuffer(raw, np.int8) if isinstance(raw, (bytes, bytearray, memoryview)) else raw
        packet = np.ascontiguousarray(source, dtype=np.int8).reshape(-1)
        if packet.nbytes != 7000:
            raise ValueError('Detection packet must contain exactly 7000 bytes')
        output = (_Detection * 300)()
        count = self.library.lds_net_detect(packet.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                  packet.nbytes, self.scales, threshold, output, 300)
        if count < 0:
            raise RuntimeError('Native LDS-Net decoder rejected the result: ' + str(count))
        return [dict(x1=float(v.x1), y1=float(v.y1), x2=float(v.x2), y2=float(v.y2),
                     score=float(v.score), category=int(v.category), anchor=int(v.anchor),
                     label=CLASS_NAMES[v.category]) for v in output[:count]]


def unpack_mask(data, full_frame=True):
    """ROI-only packed 2-bit mask; API-only expansion, never used for HDMI."""
    packed = np.frombuffer(data, np.uint8)
    if packed.size != 57600:
        raise ValueError('Expected a 57600-byte 480x480 mask')
    labels = np.stack([(packed >> (2 * i)) & 3 for i in range(4)], axis=1).reshape(480, 480)
    if np.any(labels == 3):
        raise ValueError('A published ROI contains an invalid class code')
    if not full_frame:
        return labels
    result = np.full((480, 640), 255, np.uint8)
    result[:, 80:560] = labels
    return result


def validate_package(directory):
    root = Path(directory).resolve()
    hashes = json.loads((root / 'SHA256.json').read_text(encoding='utf-8'))
    for name, expected in hashes.items():
        path = (root / name).resolve()
        try:
            path.relative_to(root)
        except ValueError:
            raise RuntimeError('LDS-Net deployment path escapes its package')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('LDS-Net deployment checksum mismatch: ' + name)
    contract = json.loads((root / 'contract.json').read_text(encoding='utf-8'))
    if contract['abi'] != ABI or tuple(contract['crop']) != CROP:
        raise RuntimeError('Unsupported LDS-Net deployment contract')
    return contract
