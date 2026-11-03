"""Versioned lossless bitmap/dense low-bit weight container and integer reader."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import shutil
import struct
from pathlib import Path
import numpy as np
import torch
from .integer_reference import pack_weights, unpack_weights, IntegerReference
from .data import write_json

HEADER = struct.Struct('<8sIIII40s')
RECORD = struct.Struct('<4I3I2BH')
MAGIC = b'RDLSP001'


def encode_weights(items):
    """items: ordered (OIHW int8 array, bits). Index is exactly 32 bytes/layer."""
    start = (HEADER.size+RECORD.size*len(items)+15)//16*16
    blob = bytearray(start)
    HEADER.pack_into(blob, 0, MAGIC, 1, len(items), HEADER.size, start, bytes(40))
    rows = []
    for i, (values, bits) in enumerate(items):
        q = np.asarray(values, dtype=np.int8)
        if q.ndim != 4 or any(d <= 0 for d in q.shape):
            raise ValueError('Nonempty OIHW shape required')
        flat = q.reshape(-1)
        dense = pack_weights(flat, bits)
        mask = flat != 0
        bitmap = np.packbits(mask, bitorder='little').tobytes()
        sparse = bitmap+pack_weights(flat[mask], bits)
        mode = int(len(sparse) < len(dense))
        payload = sparse if mode else dense
        blob.extend(bytes((-len(blob)) % 16))
        offset = len(blob)
        RECORD.pack_into(blob, HEADER.size+i*RECORD.size, *q.shape, offset, len(payload),
                         int(mask.sum()), mode, bits, 0)
        blob.extend(payload)
        rows.append({'shape': list(q.shape), 'bits': bits, 'mode': 'bitmap' if mode else 'dense',
            'offset': offset, 'bytes': len(payload), 'nnz': int(mask.sum()),
            'integer_zero_fraction': float(1-mask.mean()), 'dense_bytes': len(dense)})
    return bytes(blob), {'format': 'lds_net.sparse_weights.v1', 'file_bytes': len(blob),
        'header_bytes': HEADER.size, 'index_bytes': RECORD.size*len(items),
        'alignment_bytes': len(blob)-HEADER.size-RECORD.size*len(items)-sum(r['bytes'] for r in rows),
        'dense_weight_bytes': sum(r['dense_bytes'] for r in rows), 'layers': rows,
        'integer_zero_fraction': 1-sum(r['nnz'] for r in rows)/max(sum(math.prod(r['shape']) for r in rows),1),
        'native_sparse_execution': False}


def decode_weights(blob):
    if len(blob) < HEADER.size:
        raise ValueError('Truncated sparse header')
    magic, version, count, index, start, reserved = HEADER.unpack_from(blob)
    expected = (HEADER.size+RECORD.size*count+15)//16*16
    if magic != MAGIC or version != 1 or index != HEADER.size or start != expected or start > len(blob) or any(reserved):
        raise ValueError('Invalid sparse header')
    result = []; end = start
    for i in range(count):
        *shape, offset, length, nnz, mode, bits, reserved = RECORD.unpack_from(blob, index+i*RECORD.size)
        n = math.prod(shape)
        if not n or bits not in (2,4,8) or mode not in (0,1) or reserved or nnz > n:
            raise ValueError('Invalid sparse record')
        if offset % 16 or offset != (end+15)//16*16 or offset+length > len(blob) or any(blob[end:offset]):
            raise ValueError('Invalid sparse data bounds/alignment')
        payload = blob[offset:offset+length]
        if mode:
            bitmap_size = (n+7)//8
            expected_size = bitmap_size+(nnz*bits+7)//8
            if length != expected_size:
                raise ValueError('Sparse payload size mismatch')
            bitmap = np.unpackbits(np.frombuffer(payload[:bitmap_size],dtype=np.uint8), bitorder='little')
            if int(bitmap[:n].sum()) != nnz or bitmap[n:].any():
                raise ValueError('Invalid nonzero bitmap')
            values = unpack_weights(payload[bitmap_size:],bits,(nnz,))
            if (values == 0).any():
                raise ValueError('Zero code in nonzero value stream')
            q = np.zeros(n, np.int8); q[bitmap[:n].astype(bool)] = values
        else:
            if length != (n*bits+7)//8:
                raise ValueError('Dense payload size mismatch')
            q = unpack_weights(payload,bits,(n,))
            if int((q != 0).sum()) != nnz:
                raise ValueError('Dense nonzero count mismatch')
        result.append((q.reshape(shape), bits))
        end = offset+length
    if end != len(blob):
        raise ValueError('Unexpected trailing sparse bytes')
    return result


def graph_package(core):
    items = []; sources = []
    for row in core.nodes:
        if row['op'] != 'conv':
            continue
        op = core.ops[row['id']]
        q = (op.effective_weight().detach().cpu()/op.weight_scale.detach().cpu()[:,None,None,None]).round().clamp(
            -2**(op.bits-1), 2**(op.bits-1)-1).to(torch.int8).numpy()
        items.append((q, op.bits)); sources.append(row.get('source',row['id']))
    blob, report = encode_weights(items)
    for source, row in zip(sources, report['layers']):
        row['source'] = source
    return blob, report


def export_sparse(dense_directory, directory):
    dense_directory, directory = Path(dense_directory), Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    spec = json.loads((dense_directory/'graph.json').read_text(encoding='utf-8'))
    payload = (dense_directory/'weights.bin').read_bytes()
    convs = [r for r in spec['nodes'] if r['op']=='conv']
    items = [(unpack_weights(payload[r['weight_offset']:r['weight_offset']+r['weight_bytes']],
               r['weight_bits'],r['weight_shape']),r['weight_bits']) for r in convs]
    blob, report = encode_weights(items)
    decoded = decode_weights(blob)
    assert all(bits==other_bits and np.array_equal(q,other) for (q,bits),(other,other_bits) in zip(items,decoded))
    (directory/'weights.sparse.bin').write_bytes(blob)
    shutil.copy2(dense_directory/'parameters.npz',directory/'parameters.npz')
    spec = copy.deepcopy(spec)
    spec['format'] = 'lds_net.integer.sparse.v1'
    spec['weights_file'] = 'weights.sparse.bin'
    for i,(row,storage) in enumerate(zip([r for r in spec['nodes'] if r['op']=='conv'],report['layers'])):
        row['sparse_record'] = i
        row['dense_weight_offset'] = row.pop('weight_offset')
        row['dense_weight_bytes'] = row.pop('weight_bytes')
        storage['source'] = row.get('source',row['id'])
    report['sha256'] = hashlib.sha256(blob).hexdigest()
    spec['sparse_storage'] = report
    spec['storage']['dense_weight_payload_bytes'] = spec['storage']['weight_payload_bytes']
    spec['storage']['weight_payload_bytes'] = len(blob)
    spec['storage']['sparse_header_and_index_included'] = True
    report['unchanged_parameter_storage'] = spec['storage']
    write_json(directory/'graph.json',spec)
    write_json(directory/'storage.json',report)
    return report


class SparseIntegerReference(IntegerReference):
    """Decode to dense weights, then use the SAME verified integer operations."""
    def __init__(self, directory):
        directory = Path(directory)
        self.spec = json.loads((directory/'graph.json').read_text(encoding='utf-8'))
        if self.spec['format'] != 'lds_net.integer.sparse.v1':
            raise ValueError('Wrong sparse integer graph format')
        blob = (directory/self.spec['weights_file']).read_bytes()
        if hashlib.sha256(blob).hexdigest() != self.spec['sparse_storage']['sha256']:
            raise ValueError('Sparse weights SHA mismatch')
        decoded = decode_weights(blob)
        convs = [r for r in self.spec['nodes'] if r['op']=='conv']
        if len(decoded) != len(convs):
            raise ValueError('Sparse convolution count mismatch')
        self.weights = {}
        for row,(q,bits) in zip(convs,decoded):
            if list(q.shape) != row['weight_shape'] or bits != row['weight_bits']:
                raise ValueError('Sparse shape/precision mismatch')
            self.weights[row['id']] = torch.from_numpy(q.copy()).int()
        with np.load(directory/'parameters.npz') as archive:
            self.arrays = {k:archive[k].copy() for k in archive.files}
