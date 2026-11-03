#!/usr/bin/env python3
"""Extract selected utilization rows and preserve timing evidence, never claim full closure."""
import argparse
import json
from pathlib import Path
import re

from axis_core import digest
from axis_tool import output_path


def collect(utilization, timing):
    util = Path(utilization).read_text(encoding='utf-8-sig', errors='replace')
    timing_text = Path(timing).read_text(encoding='utf-8-sig', errors='replace')
    resources = []
    for line in util.splitlines():
        cells = [cell.strip() for cell in line.split('|')]
        if len(cells) >= 4 and re.fullmatch(r'(?:Slice LUTs|CLB LUTs|Slice Registers|CLB Registers|Block RAM Tile|DSPs|DSP48E\d*)\*?', cells[1]):
            resources.append(dict(name=cells[1], source_row=line.strip()))
    summary = []
    lines = timing_text.splitlines()
    for index, line in enumerate(lines):
        if 'WNS(ns)' in line and 'TNS(ns)' in line:
            summary = lines[index:index+5]
            break
    return dict(kind='implementation_reports', status='inconclusive', resources=resources,
                timing_summary_raw=summary, evidence={str(path): digest(path) for path in (utilization, timing)},
                warnings=['Selected raw report rows only. Check report stage/device, unconstrained paths, hold, pulse width, CDC and exceptions; positive WNS alone is not timing closure',
                          'Missing rows mean unsupported report layout; do not invent utilization numbers'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--utilization', required=True)
    parser.add_argument('--timing', required=True)
    parser.add_argument('--workspace', default='.')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = collect(args.utilization, args.timing)
    output_path(args.output, args.workspace).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Collected %d resource rows; timing closure is not inferred.' % len(result['resources']))
