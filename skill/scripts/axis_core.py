"""Standard-library AXI-Stream evidence analysis. Python 3.8+, no hardware imports."""
import csv
import hashlib
import json
import math
from pathlib import Path


def integer(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    value = str(value).strip().lower().replace('_', '')
    if not value or any(c in value for c in 'xz?') and not value.startswith('0x'):
        raise ValueError('Unknown/empty integer: %r' % value)
    return int(value, 16 if value.startswith('0x') else 2 if value.startswith('0b') else 10)


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('%s must be finite and positive' % name)
    return value


def contract(config):
    if config.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    stream, capture = config['stream'], config['capture']
    width = stream['data_width_bits']
    if type(width) is not int or width <= 0 or width % 8:
        raise ValueError('data_width_bits must be a positive multiple of 8')
    if stream['framing'] not in ('packet', 'video', 'continuous'):
        raise ValueError('framing must be packet/video/continuous')
    if capture['start'] not in ('boundary', 'partial'):
        raise ValueError('capture.start must be boundary/partial')
    for key in ('contiguous', 'complete_end'):
        if type(capture[key]) is not bool:
            raise ValueError('capture.%s must be boolean' % key)
    if capture.get('clock_hz') is not None:
        positive(capture['clock_hz'], 'clock_hz')
    signals = config['signals']
    allowed = {'cycle', 'resetn', 'valid', 'ready', 'data', 'keep', 'strb', 'last', 'user', 'id', 'dest'}
    if set(signals) - allowed or any(not isinstance(v, str) or not v for v in signals.values()):
        raise ValueError('Invalid signal keys/column names')
    if signals.get('id') or signals.get('dest'):
        if stream.get('interleaved', False):
            raise ValueError('Interleaved streams require a per-stream adapter; unsupported by this checker')
    for name in ('cycle', 'valid', 'ready', 'data', 'resetn'):
        if not isinstance(signals.get(name), str) or not signals[name]:
            raise ValueError('signals.%s is required' % name)
    if stream['framing'] != 'continuous' and not signals.get('last'):
        raise ValueError('Packet/video checking requires last')
    if stream['framing'] == 'video':
        if not signals.get('user'):
            raise ValueError('Video checking requires user (bit 0 is SOF)')
        for name in ('beats_per_line', 'lines_per_frame'):
            if type(stream.get(name)) is not int or stream[name] <= 0:
                raise ValueError('video requires positive integer %s' % name)
    for name in ('packet_bytes', 'expected_packets', 'expected_frames', 'max_no_handshake_cycles'):
        if stream.get(name) is not None and (type(stream[name]) is not int or stream[name] <= 0):
            raise ValueError('%s must be a positive integer' % name)
    if len(set(signals.values())) != len(signals):
        raise ValueError('Signal mappings must use distinct columns')
    return config


def read_trace(path, config):
    signals = config['signals']
    with open(path, encoding='utf-8-sig', newline='') as handle:
        reader = csv.DictReader(handle)
        missing = set(signals.values()) - set(reader.fieldnames or [])
        if missing:
            raise ValueError('Missing CSV columns: %s' % ', '.join(sorted(missing)))
        for line, row in enumerate(reader, 2):
            try:
                sample = {key: integer(row[col]) for key, col in signals.items()
                          if key in ('cycle', 'resetn', 'valid', 'ready')}
                for key in signals:
                    if key not in sample:
                        # Payload is undefined when TVALID=0 (including reset).
                        sample[key] = integer(row[signals[key]]) if sample['valid'] and sample['resetn'] else 0
                sample.setdefault('keep', (1 << (config['stream']['data_width_bits'] // 8)) - 1)
                sample.setdefault('strb', sample['keep'])
                sample.setdefault('last', 0)
                sample.setdefault('user', 0)
                yield sample
            except (ValueError, TypeError) as exc:
                raise ValueError('CSV line %d: %s' % (line, exc)) from exc


def analyze(samples, config):
    contract(config)
    stream, capture = config['stream'], config['capture']
    lanes = stream['data_width_bits'] // 8
    failures, warnings, packets = [], [], []
    counts = dict(active_cycles=0, reset_cycles=0, handshakes=0, valid_bytes=0,
                  data_bytes=0, backpressure_cycles=0, source_bubbles=0, both_low=0,
                  gaps=0, completed_frames=0)
    previous = pending = None
    packet_size = packet_beats = line_beats = frame_lines = 0
    in_frame = False
    synced = capture['start'] == 'boundary'
    no_fire = max_stall = stall = 0
    window_first = window_last = None

    def fail(rule, cycle, detail):
        category = ('application_liveness' if rule == 'APPLICATION_NO_PROGRESS' else
                    'protocol_subset' if rule in ('VALID_DURING_RESET','STALL_STABILITY','STRB_WITHOUT_KEEP') else 'interface_contract')
        actions = {
            'STALL_STABILITY': 'Check output holding register and advance counters only on handshake; inspect the first changed beat',
            'VALID_DURING_RESET': 'Check reset polarity and the source valid reset path',
            'STRB_WITHOUT_KEEP': 'Review byte qualifier generation and endpoint-specific packing',
            'PACKET_LENGTH': 'Compare accepted TKEEP byte count against the contract; inspect early/late TLAST and width conversion',
            'MISSING_PACKET_END': 'Confirm capture completeness, then inspect whether the final TLAST beat is generated and accepted',
            'VIDEO_EOL': 'Check accepted line-beat counter, pixels-per-beat and TLAST generation',
            'EARLY_SOF': 'Inspect frame drop/overflow/reset counters and the previous incomplete frame',
            'MISSING_SOF': 'Verify accepted TUSER[0] at the first beat and confirm the capture start declaration',
            'APPLICATION_NO_PROGRESS': 'Check both neighboring interfaces, FIFO occupancy, source availability and receiver startup; sustained backpressure can be legal',
        }
        failures.append(dict(rule=rule, cycle=cycle, category=category, detail=detail,
                             next_check=actions.get(rule,'Verify capture boundaries and accepted transaction counts against the interface contract')))

    for supplied in samples:
        sample = dict(supplied)
        sample.setdefault('keep', (1 << lanes) - 1)
        sample.setdefault('strb', sample['keep'])
        sample.setdefault('last', 0)
        sample.setdefault('user', 0)
        cycle = sample['cycle']
        if cycle < 0 or any(sample[key] not in (0, 1) for key in ('resetn', 'valid', 'ready', 'last')):
            raise ValueError('cycle must be nonnegative; resetn/valid/ready/last must be 0/1')
        if previous is not None:
            if cycle <= previous:
                raise ValueError('cycle must be strictly increasing')
            if cycle != previous + 1:
                if capture['contiguous']:
                    raise ValueError('Gap in contiguous trace at cycle %d' % cycle)
                counts['gaps'] += 1
                pending = None
                synced = False
                packet_size = packet_beats = line_beats = frame_lines = no_fire = stall = 0
                in_frame = False
        previous = cycle
        if window_first is None:
            window_first = cycle
        window_last = cycle
        if not sample['resetn']:
            counts['reset_cycles'] += 1
            if sample['valid']:
                fail('VALID_DURING_RESET', cycle, 'TVALID must be low during reset')
            pending = None
            packet_size = packet_beats = line_beats = frame_lines = no_fire = stall = 0
            in_frame = False
            synced = True
            continue
        if sample['valid']:
            if not 0 <= sample['data'] < (1 << stream['data_width_bits']):
                raise ValueError('TDATA exceeds configured width at cycle %d' % cycle)
            if not 0 <= sample['keep'] < (1 << lanes) or not 0 <= sample['strb'] < (1 << lanes):
                raise ValueError('TKEEP/TSTRB exceeds lane width')
            if sample['strb'] & ~sample['keep']:
                fail('STRB_WITHOUT_KEEP', cycle, 'TSTRB=1 with TKEEP=0 is reserved')
        counts['active_cycles'] += 1
        if pending is not None:
            changed = [key for key in ('keep', 'strb', 'last', 'user', 'id', 'dest')
                       if key in pending and sample.get(key) != pending[key]]
            # Null/position byte data are not compared; qualifier stability is checked separately.
            mask = sum(255 << (8 * lane) for lane in range(lanes)
                       if (pending['keep'] & pending['strb']) >> lane & 1)
            if not sample['valid'] or changed or (sample['data'] ^ pending['data']) & mask:
                fail('STALL_STABILITY', cycle, 'Pending beat changed/withdrawn; fields=%s' % changed)
        fire = bool(sample['valid'] and sample['ready'])
        pending = dict(sample) if sample['valid'] and not sample['ready'] else None
        if sample['valid'] and not sample['ready']:
            counts['backpressure_cycles'] += 1
            stall += 1
            max_stall = max(max_stall, stall)
        else:
            stall = 0
        if not sample['valid']:
            counts['source_bubbles' if sample['ready'] else 'both_low'] += 1
        no_fire = 0 if fire else no_fire + 1
        limit = stream.get('max_no_handshake_cycles')
        if limit and no_fire == limit + 1:
            fail('APPLICATION_NO_PROGRESS', cycle, 'Application progress deadline exceeded; not a protocol violation')
        if not fire:
            continue
        counts['handshakes'] += 1
        nbytes = bin(sample['keep']).count('1')
        counts['valid_bytes'] += nbytes
        counts['data_bytes'] += bin(sample['keep'] & sample['strb']).count('1')
        if stream['framing'] == 'packet':
            packet_size += nbytes
            packet_beats += 1
            if sample['last']:
                packets.append(dict(end_cycle=cycle, valid_bytes=packet_size, beats=packet_beats,
                                    complete=synced))
                if synced and stream.get('packet_bytes') is not None and packet_size != stream['packet_bytes']:
                    fail('PACKET_LENGTH', cycle, 'Expected %d bytes, received %d' % (stream['packet_bytes'], packet_size))
                synced = True
                packet_size = packet_beats = 0
        elif stream['framing'] == 'video':
            if sample['user'] & 1:
                if in_frame:
                    fail('EARLY_SOF', cycle, 'New SOF interrupted an incomplete frame')
                line_beats = frame_lines = 0
                in_frame = True
                synced = True
            if in_frame:
                line_beats += 1
                expected_last = line_beats == stream['beats_per_line']
                if bool(sample['last']) != expected_last:
                    fail('VIDEO_EOL', cycle, 'TLAST must match the configured last beat of each line')
                if sample['last']:
                    frame_lines += 1
                    line_beats = 0
                    if frame_lines == stream['lines_per_frame']:
                        counts['completed_frames'] += 1
                        in_frame = False
                elif line_beats > stream['beats_per_line']:
                    # Continue collecting evidence without silently accepting an overlong line.
                    pass
            elif not (sample['user'] & 1) and synced:
                fail('MISSING_SOF', cycle, 'Accepted video beat outside a frame')
    if previous is None:
        raise ValueError('Empty trace')
    if capture['complete_end'] and capture['contiguous']:
        if pending:
            fail('CAPTURE_PENDING_BEAT', previous, 'Declared complete capture ends before an offered beat is accepted')
        if stream['framing'] == 'packet' and packet_beats:
            fail('MISSING_PACKET_END', previous, 'Declared complete capture ends with an unterminated packet')
        if in_frame:
            fail('INCOMPLETE_VIDEO_FRAME', previous, 'Declared complete capture ends inside a frame')
    elif pending or packet_beats or in_frame:
        warnings.append('Capture ends during activity: cannot infer missing TLAST from truncation')
    if not capture['contiguous']:
        warnings.append('Sparse capture: only adjacent samples support stability; no wall-clock bandwidth is reported')
    if capture['start'] == 'partial':
        warnings.append('Partial start: first packet is excluded from length checks; video waits for accepted SOF')
    if capture['complete_end'] and capture['start'] == 'boundary' and capture['contiguous']:
        for key, actual in (('expected_packets', len(packets)), ('expected_frames', counts['completed_frames'])):
            if stream.get(key) is not None and actual != stream[key]:
                fail('EXPECTED_COUNT', previous, '%s expected %d, observed %d' % (key, stream[key], actual))
    active = counts['active_cycles']
    metrics = dict(handshake_utilization=counts['handshakes'] / active if active else None,
                   backpressure_ratio=counts['backpressure_cycles'] / active if active else None,
                   longest_backpressure_cycles=max_stall, active_bandwidth_Bps=None,
                   window_bandwidth_Bps=None, ideal_bus_Bps=None)
    if not capture['contiguous']:
        metrics['handshake_utilization'] = None
        metrics['backpressure_ratio'] = None
    hz = capture.get('clock_hz')
    if hz and capture['contiguous']:
        metrics['active_bandwidth_Bps'] = counts['valid_bytes'] * hz / active if active else None
        metrics['window_bandwidth_Bps'] = counts['valid_bytes'] * hz / (window_last - window_first + 1)
        metrics['ideal_bus_Bps'] = lanes * hz
    limited = bool(warnings or counts['active_cycles'] == 0 or counts['handshakes'] == 0)
    if not counts['handshakes']:
        warnings.append('No accepted beats: the capture cannot validate a data transfer')
    if stream['framing'] == 'packet' and not capture['complete_end'] and not packets:
        warnings.append('No packet boundary observed')
    return dict(kind='axis_trace', status='fail' if failures else 'inconclusive' if limited else 'pass',
                counts=counts, metrics=metrics, packets=packets, failures=failures, warnings=warnings,
                counts_basis='clock_cycles' if capture['contiguous'] else 'observed_samples_only; runs are lower bounds',
                scope='Observed single-clock trace only; no CDC proof, RTL dependency proof, or unobserved end-to-end integrity claim')


def extract_payload(samples, width_bits):
    """Pack accepted data bytes in lane order, excluding null and position bytes."""
    output=bytearray()
    for sample in samples:
        if sample['resetn'] and sample['valid'] and sample['ready']:
            for lane in range(width_bits//8):
                if (sample['keep'] & sample['strb']) >> lane & 1:
                    output.append((sample['data'] >> (8*lane)) & 255)
    return bytes(output)


STATUS_BITS = {0: 'Halted', 1: 'Idle', 3: 'SGIncld', 4: 'DMAIntErr', 5: 'DMASlvErr',
               6: 'DMADecErr', 8: 'SGIntErr', 9: 'SGSlvErr', 10: 'SGDecErr',
               12: 'IOC_Irq', 13: 'Dly_Irq', 14: 'Err_Irq'}
REGISTERS = {'MM2S_DMACR': 0x00, 'MM2S_DMASR': 0x04, 'MM2S_SA': 0x18,
             'MM2S_SA_MSB': 0x1c, 'MM2S_LENGTH': 0x28, 'S2MM_DMACR': 0x30,
             'S2MM_DMASR': 0x34, 'S2MM_DA': 0x48, 'S2MM_DA_MSB': 0x4c, 'S2MM_LENGTH': 0x58}


def decode_dma(snapshot):
    if snapshot.get('ip_type') != 'axi_dma':
        raise ValueError('Only AXI DMA PG021 is decoded; VDMA/custom DMA require another register map')
    result = {}
    for direction in ('MM2S', 'S2MM'):
        key = direction + '_DMASR'
        if key not in snapshot['registers']:
            continue
        value = integer(snapshot['registers'][key])
        if not 0 <= value <= 0xffffffff:
            raise ValueError('DMASR must be uint32')
        flags = [name for bit, name in STATUS_BITS.items() if value & (1 << bit)]
        result[direction] = dict(raw='0x%08x' % value, flags=flags,
                                 errors=[name for name in flags if name.endswith('Err')],
                                 halted=bool(value & 1), idle=bool(value & 2), sg_included=bool(value & 8))
    if not result:
        raise ValueError('Snapshot contains no DMA status registers')
    errors = any(channel['errors'] for channel in result.values())
    candidates = []
    for direction, channel in result.items():
        for error in channel['errors']:
            candidates.append(dict(channel=direction, error=error, next_check={
                'DMADecErr': 'Verify device address, address width and exported interconnect mapping',
                'DMASlvErr': 'Inspect memory-map slave response, access path and reset state',
                'DMAIntErr': 'Check actual IP mode, receive capacity, packet length and PG021 error conditions',
            }.get(error, 'Inspect SG descriptor ownership, address and memory response; Simple live adapter refuses SG')))
    return dict(kind='dma_snapshot', status='fail' if errors else 'inconclusive', channels=result, candidates=candidates,
                warnings=['One snapshot cannot prove progress, packet completion, cache consistency or a TLAST root cause'])


def compare_binary(expected, actual):
    expected, actual = Path(expected).read_bytes(), Path(actual).read_bytes()
    differences = [i for i, (a, b) in enumerate(zip(expected, actual)) if a != b]
    first = differences[0] if differences else min(len(expected), len(actual)) if len(expected) != len(actual) else None
    mismatch = len(differences) + abs(len(expected) - len(actual))
    return dict(kind='binary_compare', status='fail' if mismatch else 'pass', expected_bytes=len(expected),
                actual_bytes=len(actual), mismatch_bytes=mismatch, first_mismatch_offset=first,
                expected_context=expected[max(0, (first or 0)-8):(first or 0)+9].hex() if first is not None else None,
                actual_context=actual[max(0, (first or 0)-8):(first or 0)+9].hex() if first is not None else None)


def summarize_benchmark(rows, warmup=0):
    if type(warmup) is not int or warmup < 0:
        raise ValueError('warmup must be a nonnegative integer')
    rows = list(rows)[warmup:]
    if not rows:
        raise ValueError('No samples after warmup')
    groups = {}
    for row in rows:
        scope = row['scope']
        if scope not in ('python_e2e', 'dma', 'hardware'):
            raise ValueError('scope must be python_e2e/dma/hardware')
        duration = positive(float(row['seconds']), 'seconds')
        size = integer(row['bytes'])
        if size <= 0:
            raise ValueError('bytes must be positive')
        groups.setdefault((scope, size), []).append(duration)
    results = []
    for (scope, size), durations in sorted(groups.items()):
        ordered = sorted(durations)
        def percentile(fraction):
            index = (len(ordered) - 1) * fraction
            lower = int(index)
            upper = min(lower + 1, len(ordered) - 1)
            return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)
        results.append(dict(scope=scope, bytes=size, samples=len(ordered), min_s=ordered[0],
                            median_s=percentile(.5), p95_s=percentile(.95), max_s=ordered[-1],
                            aggregate_Bps=size * len(ordered) / sum(ordered)))
    return dict(kind='benchmark', status='pass', warmup_discarded=warmup, groups=results,
                warnings=['Timing scope is supplied by the caller; bytes are one-way payload, not TX+RX sum'])


def render_report(results):
    lines = ['# PYNQ AXI4-Stream 证据报告', '',
             '结论仅限所列输入、观测窗口及配置；候选原因需要额外验证。', '',
             '| 检查 | 状态 | 证据级别 | 失败项数 |', '|---|---|---|---|']
    for result in results:
        lines.append('| %s | %s | %s | %d |' % (result.get('kind','unknown'),result.get('status','unknown'),
                     result.get('evidence_level','由输入来源确认'),len(result.get('failures',[]))))
    lines.append('')
    for result in results:
        lines.extend(['## %s — %s' % (result.get('kind', 'unknown'), result.get('status', 'unknown')), ''])
        for failure in result.get('failures', []):
            lines.append('- 周期 %s / %s：%s' % (failure['cycle'], failure['rule'], failure['detail']))
            if failure.get('next_check'):
                lines.append('  - 验证动作（%s）：%s' % (failure['category'],failure['next_check']))
        if result.get('kind') == 'benchmark':
            lines.extend(['', '| scope | 字节 | 样本 | median(s) | P95(s) | min/max(s) | B/s |', '|---|---|---|---|---|---|---|'])
            for group in result['groups']:
                lines.append('| %s | %d | %d | %.6g | %.6g | %.6g / %.6g | %.6g |' % (
                    group['scope'],group['bytes'],group['samples'],group['median_s'],group['p95_s'],
                    group['min_s'],group['max_s'],group['aggregate_Bps']))
        for warning in result.get('warnings', []):
            lines.append('- 限制：%s' % warning)
        for candidate in result.get('candidates', []):
            lines.append('- 下一步：%s' % candidate)
        lines.extend(['', '```json', json.dumps(result, ensure_ascii=False, indent=2), '```', ''])
    lines.extend(['## 定位与复测记录', '',
                  '- 候选原因：结合 failures、DMA 错误位和接口契约列出；无证据不下根因结论。',
                  '- 最小补采：异常接口及相邻接口的握手、边带、时钟复位与 DMA 前后状态。',
                  '- 修复验证：同种子故障复现、正常反压对照、黄金数据比对及性能复测。',
                  '- 板上/跨板验证：只有附设备、版本、运行记录和原始证据后才可标记已验证。', ''])
    return '\n'.join(lines)
