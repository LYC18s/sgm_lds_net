#!/usr/bin/env python3
"""Deterministic synthetic evidence; never label these traces as board captures."""
import argparse
import copy
import csv
import json
from pathlib import Path
import random

from axis_tool import output_path


FIELDS = ('cycle', 'resetn', 'valid', 'ready', 'data', 'keep', 'last', 'user')


def packet_rows(seed=51, packets=2):
    rng = random.Random(seed)
    rows = [dict(zip(FIELDS, (0, 0, 0, 0, 0, 0, 0, 0)))]
    def emit(valid=0, ready=1, data=0, keep=15, last=0):
        rows.append(dict(zip(FIELDS, (len(rows), 1, valid, ready, data, keep, last, 0))))
    for packet in range(packets):
        for beat in range(3):
            for _ in range(rng.randrange(3)):
                emit(ready=1)
            data = (packet * 3 + beat + 1) * 0x01010101
            keep, last = (3, 1) if beat == 2 else (15, 0)
            for _ in range(rng.randrange(1, 5)):
                emit(1, 0, data, keep, last)
            emit(1, 1, data, keep, last)
    emit()
    return rows


def video_rows():
    rows = [dict(zip(FIELDS, (0, 0, 0, 0, 0, 0, 0, 0)))]
    for frame in range(2):
        for line in range(2):
            for beat in range(4):
                row = dict(zip(FIELDS, (len(rows), 1, 1, 0, frame * 16 + line * 4 + beat, 7,
                                       int(beat == 3), int(line == 0 and beat == 0))))
                rows.append(row)
                row = dict(row, cycle=len(rows), ready=1)
                rows.append(row)
    return rows


def generate(directory, workspace):
    root = Path(__file__).resolve().parents[1]
    packet_config = json.loads((root/'assets/packet.json').read_text(encoding='utf-8'))
    video_config = json.loads((root/'assets/video.json').read_text(encoding='utf-8'))
    good = packet_rows()
    cases = []
    def case(name, rows, config, expected, rule=None):
        path = output_path(Path(directory)/(name+'.csv'), workspace)
        with path.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        output_path(Path(directory)/(name+'.json'), workspace).write_text(
            json.dumps(config, indent=2)+'\n', encoding='utf-8')
        cases.append(dict(name=name, expected=expected, rule=rule, evidence_level='synthetic'))
    case('packet_ok', good, packet_config, 'pass')
    bad = copy.deepcopy(good)
    index = next(i for i in range(2, len(bad)) if bad[i-1]['valid'] and not bad[i-1]['ready'])
    bad[index]['data'] ^= 1
    case('stalled_data_changed', bad, packet_config, 'fail', 'STALL_STABILITY')
    bad = copy.deepcopy(good)
    bad[index]['valid'] = 0
    case('valid_withdrawn', bad, packet_config, 'fail', 'STALL_STABILITY')
    bad = copy.deepcopy(good)
    for row in bad:
        row['last'] = 0
    case('missing_tlast', bad, packet_config, 'fail', 'MISSING_PACKET_END')
    bad = copy.deepcopy(good)
    index = next(i for i, r in enumerate(bad) if r['valid'] and r['ready'])
    for i in range(1, index+1):
        if bad[i]['valid']:
            bad[i]['last'] = 1
    case('early_tlast', bad, packet_config, 'fail', 'PACKET_LENGTH')
    bad = copy.deepcopy(good)
    for row in bad:
        if row['last']:
            row['keep'] = 15
    case('wrong_tail_keep', bad, packet_config, 'fail', 'PACKET_LENGTH')
    conf = copy.deepcopy(packet_config)
    conf['capture']['complete_end'] = False
    case('truncated_capture', good[:8], conf, 'inconclusive')
    conf = copy.deepcopy(packet_config)
    conf['capture']['contiguous'] = False
    case('sparse_capture', [r for r in good if r['valid'] and r['ready']], conf, 'inconclusive')
    conf = copy.deepcopy(packet_config)
    conf['capture']['start'] = 'partial'
    # Begin with one final, accepted beat of an already active packet.
    boundary = next(i for i, row in enumerate(good) if row['last'] and row['ready'])
    partial = copy.deepcopy(good[boundary:])
    for i, row in enumerate(partial):
        row['cycle'] = i
    case('partial_start', partial, conf, 'inconclusive')
    conf = copy.deepcopy(packet_config)
    conf['stream']['max_no_handshake_cycles'] = 3
    stalled = [dict(zip(FIELDS, (i, 1, 1, 0, 42, 15, 0, 0))) for i in range(10)]
    case('application_timeout', stalled, conf, 'fail', 'APPLICATION_NO_PROGRESS')
    case('video_ok', video_rows(), video_config, 'pass')
    bad = video_rows()
    for row in bad:
        if row['last']:
            row['last'] = 0
    case('video_no_eol', bad, video_config, 'fail', 'VIDEO_EOL')
    output_path(Path(directory)/'manifest.json', workspace).write_text(
        json.dumps(dict(seed=51, cases=cases), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    for name, value in (
        ('dma_idle.json', {'ip_type': 'axi_dma', 'registers': {'MM2S_DMASR': '0x1002', 'S2MM_DMASR': '0x1002'}}),
        ('dma_decode_error.json', {'ip_type': 'axi_dma', 'registers': {'S2MM_DMASR': '0x4041'}})):
        output_path(Path(directory)/name, workspace).write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')
    output_path(Path(directory)/'timings.csv', workspace).write_text(
        'scope,bytes,seconds\npython_e2e,4096,0.1\npython_e2e,4096,0.002\npython_e2e,4096,0.003\npython_e2e,4096,0.004\n', encoding='utf-8')
    payload = b''.join(bytes([i])*4 for i in (1, 2)) + bytes([3])*2
    payload += b''.join(bytes([i])*4 for i in (4, 5)) + bytes([6])*2
    output_path(Path(directory)/'expected.bin', workspace).write_bytes(payload)
    output_path(Path(directory)/'actual_ok.bin', workspace).write_bytes(payload)
    damaged = bytearray(payload)
    damaged[9] ^= 1
    output_path(Path(directory)/'actual_bad.bin', workspace).write_bytes(damaged)
    return cases


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default='skill/examples/generated')
    parser.add_argument('--workspace', default='.')
    args = parser.parse_args()
    print(json.dumps(generate(args.directory, args.workspace), indent=2))
