#!/usr/bin/env python3
"""Offline CLI; no third-party dependencies, hardware access or implicit writes."""
import argparse
import csv
import importlib.util
import json
import platform
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from axis_core import (analyze, compare_binary, contract, decode_dma, digest, extract_payload, load_json,
                       read_trace, render_report, summarize_benchmark)


def output_path(path, workspace):
    root, target = Path(workspace).resolve(), Path(path).resolve()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise ValueError('Output must stay inside workspace: %s' % root) from exc
    if target == root:
        raise ValueError('Output must be a file')
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def inspect_hwh(path):
    raw = Path(path).read_bytes()
    if len(raw) > 32 * 1024 * 1024 or b'<!DOCTYPE' in raw.upper() or b'<!ENTITY' in raw.upper():
        raise ValueError('HWH is oversized or contains a DTD/entity declaration')
    root = ET.fromstring(raw)
    modules = []
    for element in root.iter():
        if element.tag.split('}')[-1].upper() == 'MODULE':
            parameters = {p.attrib['NAME']: p.attrib.get('VALUE') for p in element.iter()
                          if p.tag.split('}')[-1].upper() == 'PARAMETER' and 'NAME' in p.attrib}
            modules.append(dict(name=element.attrib.get('FULLNAME', element.attrib.get('INSTANCE')),
                                vlnv=element.attrib.get('VLNV'), parameters=parameters))
    return dict(kind='hwh_inventory', status='inconclusive', modules=modules,
                source_sha256=digest(path), warnings=['Metadata describes exported hardware, not proof of the currently loaded bitstream'])


def environment():
    packages = {}
    for name in ('pynq', 'numpy'):
        spec = importlib.util.find_spec(name)
        packages[name] = spec.origin if spec else None
    return dict(kind='environment', status='inconclusive', python=sys.version,
                platform=platform.platform(), packages=packages,
                warnings=['No package import, Overlay download, MMIO access or hardware mutation was performed'])


def convert_vcd(source, mapping, destination):
    """Sample values immediately BEFORE each rising edge; handles ASCII VCD only."""
    if 'clock' not in mapping:
        raise ValueError('VCD map requires clock')
    with open(source, encoding='utf-8-sig') as handle:
        tokens = iter(handle.read().split())
    scope, variables, values = [], {}, {}
    for token in tokens:
        if token == '$scope':
            next(tokens)
            scope.append(next(tokens))
        elif token == '$upscope':
            scope.pop()
        elif token == '$var':
            next(tokens)
            width, code, name = int(next(tokens)), next(tokens), next(tokens)
            variables['.'.join(scope + [name])] = (code, width)
        elif token == '$enddefinitions':
            break
    missing = set(mapping.values()) - set(variables)
    if missing:
        raise ValueError('VCD signals not found (exact hierarchy required): %s' % sorted(missing))
    selected = {name: variables[path][0] for name, path in mapping.items()}
    columns = ['cycle'] + [name for name in mapping if name != 'clock']
    cycle, changes, skip_directive = 0, {}, False
    edge_rows = []

    def commit():
        nonlocal cycle
        clock = selected['clock']
        if values.get(clock) == '0' and changes.get(clock) == '1':
            row = {'cycle': cycle}
            for name, code in selected.items():
                if name == 'clock':
                    continue
                value = values.get(code, 'x')
                row[name] = value if any(c in value.lower() for c in 'xz') else '0b' + value
            edge_rows.append(row)
            cycle += 1
        values.update(changes)
        changes.clear()

    for token in tokens:
        if token.startswith('#'):
            commit()
        elif token in ('$dumpvars', '$dumpall', '$dumpon'):
            continue
        elif token == '$end':
            skip_directive = False
        elif token == '$dumpoff':
            raise ValueError('VCD dumpoff prevents continuous-cycle analysis; export an unfiltered window')
        elif token.startswith('$'):
            skip_directive = True
        elif skip_directive:
            continue
        elif token[0] in '01xXzZ':
            changes[token[1:]] = token[0].lower()
        elif token[0] in 'bB':
            changes[next(tokens)] = token[1:].lower()
        elif token[0] in 'rR':
            next(tokens)
    commit()
    if not edge_rows:
        raise ValueError('No known 0->1 clock edges in VCD')
    with open(destination, 'w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(edge_rows)
    return dict(kind='vcd_conversion', status='pass', samples=cycle, sampling='pre-edge',
                warnings=['VCD must contain all clock edges and dump activity; dumpoff gaps are not a continuous capture'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='.', help='Only output boundary, defaults to current directory')
    parser.add_argument('--output', help='JSON/Markdown output; absent means stdout')
    commands = parser.add_subparsers(dest='command', required=True)
    check = commands.add_parser('check', help='Analyze normalized ILA/CSV trace')
    check.add_argument('trace')
    check.add_argument('--config', required=True)
    vcd = commands.add_parser('vcd', help='Convert ASCII VCD to canonical CSV')
    vcd.add_argument('source')
    vcd.add_argument('--map', required=True)
    vcd.add_argument('--csv', required=True)
    decode = commands.add_parser('decode', help='Decode an offline AXI DMA snapshot')
    decode.add_argument('snapshot')
    compare = commands.add_parser('compare', help='Exact comparison of packed payload files')
    compare.add_argument('expected')
    compare.add_argument('actual')
    extract = commands.add_parser('extract', help='Pack accepted data-byte lanes into a binary reference-comparison input')
    extract.add_argument('trace')
    extract.add_argument('--config', required=True)
    extract.add_argument('--binary', required=True)
    bench = commands.add_parser('benchmark', help='Summarize measurements with explicit timing scope')
    bench.add_argument('csv')
    bench.add_argument('--warmup', type=int, default=0)
    hwh = commands.add_parser('hwh', help='Read local hardware metadata without PYNQ')
    hwh.add_argument('path')
    commands.add_parser('env', help='Read Python/package availability without importing PYNQ')
    report = commands.add_parser('report', help='Combine JSON evidence into Markdown')
    report.add_argument('results', nargs='+')
    args = parser.parse_args(argv)
    try:
        inputs = []
        for name in ('trace','config','source','map','snapshot','expected','actual','path'):
            value = getattr(args,name,None)
            if value:
                inputs.append(Path(value).resolve())
        if args.command == 'benchmark':
            inputs.append(Path(args.csv).resolve())
        inputs.extend(Path(value).resolve() for value in getattr(args,'results',[]))
        outputs = [Path(value).resolve() for value in (args.output,getattr(args,'binary',None),
                   args.csv if args.command == 'vcd' else None) if value]
        if len(set(outputs)) != len(outputs) or any(path in inputs for path in outputs):
            raise ValueError('Outputs must be distinct and must not overwrite an input')
        if args.command == 'check':
            config = contract(load_json(args.config))
            result = analyze(read_trace(args.trace, config), config)
            result['evidence'] = {str(args.trace): digest(args.trace), str(args.config): digest(args.config)}
        elif args.command == 'decode':
            result = decode_dma(load_json(args.snapshot))
            result['evidence'] = {str(args.snapshot): digest(args.snapshot)}
        elif args.command == 'compare':
            result = compare_binary(args.expected, args.actual)
            result['evidence'] = {str(path): digest(path) for path in (args.expected, args.actual)}
        elif args.command == 'extract':
            config = contract(load_json(args.config))
            checked = analyze(read_trace(args.trace, config), config)
            binary = output_path(args.binary, args.workspace)
            binary.write_bytes(extract_payload(read_trace(args.trace,config),config['stream']['data_width_bits']))
            result = dict(kind='payload_extraction',status=checked['status'],data_bytes=binary.stat().st_size,
                          evidence={str(path):digest(path) for path in (args.trace,args.config,binary)},
                          failures=checked['failures'],warnings=checked['warnings']+
                          ['Lane order is low-to-high byte lane; no algorithm correctness claim until compared against an independent reference'])
        elif args.command == 'benchmark':
            with open(args.csv, encoding='utf-8-sig', newline='') as handle:
                result = summarize_benchmark(csv.DictReader(handle), args.warmup)
            result['evidence'] = {str(args.csv): digest(args.csv)}
        elif args.command == 'hwh':
            result = inspect_hwh(args.path)
        elif args.command == 'env':
            result = environment()
        elif args.command == 'vcd':
            result = convert_vcd(args.source, load_json(args.map), output_path(args.csv, args.workspace))
        else:
            results = [load_json(path) for path in args.results]
            result = {'status': 'fail' if any(r.get('status') == 'fail' for r in results)
                      else 'inconclusive' if any(r.get('status') != 'pass' for r in results) else 'pass'}
        text = render_report(results) if args.command == 'report' else json.dumps(result, ensure_ascii=False, indent=2)
        if args.output:
            output_path(args.output, args.workspace).write_text(text + '\n', encoding='utf-8')
        else:
            print(text)
        return {'pass': 0, 'fail': 1, 'inconclusive': 3}[result['status']]
    except (ValueError, KeyError, TypeError, OSError, ET.ParseError, StopIteration) as exc:
        print('Input/configuration error: %s' % exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
