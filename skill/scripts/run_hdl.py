#!/usr/bin/env python3
"""Run the portable bench only with explicitly supplied existing Icarus executables."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from axis_tool import output_path


def run(iverilog, vvp, directory, workspace):
    root = Path(__file__).resolve().parents[1]
    target = output_path(Path(directory)/'bench.vvp', workspace)
    env = os.environ.copy()
    temp = output_path(Path(directory)/'tmp'/'reserved', workspace).parent
    # Child-process variables only; no user/global environment is changed.
    env.update(TEMP=str(temp), TMP=str(temp), TMPDIR=str(temp))
    commands = [([str(Path(iverilog).resolve()), '-g2012', '-s', 'tb_axis', '-o', target.name,
                  str(root/'assets/axis_checker.sv'), str(root/'assets/tb_axis.sv')], 'compile'),
                ([str(Path(vvp).resolve()), target.name], 'normal'),
                ([str(Path(vvp).resolve()), target.name, '+FAULT'], 'fault')]
    stages = []
    for command, name in commands:
        result = subprocess.run(command, cwd=target.parent, env=env, capture_output=True, text=True, timeout=60)
        log = result.stdout+'\n'+result.stderr
        output_path(Path(directory)/(name+'.log'), workspace).write_text(log, encoding='utf-8')
        passed = result.returncode == 0 and (name == 'compile' or 'PASS: axis checker' in log)
        if name == 'normal':
            passed = passed and 'AXIS_ERROR' not in log and 'errors=0' in log
        if name == 'fault':
            passed = passed and 'AXIS_ERROR STALL_DATA' in log
        stages.append(dict(name=name, passed=passed, command=command, exit_code=result.returncode))
        if name != 'compile' and (target.parent/'axis.vcd').exists():
            # Preserve both runs for trace-parser cross-checks.
            output_path(Path(directory)/(name+'.vcd'), workspace).write_bytes((target.parent/'axis.vcd').read_bytes())
        if not passed:
            break
    record = dict(kind='hdl_validation', status='pass' if len(stages)==3 and all(s['passed'] for s in stages) else 'fail', stages=stages)
    output_path(Path(directory)/'validation.json', workspace).write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iverilog', required=True)
    parser.add_argument('--vvp', required=True)
    parser.add_argument('--directory', default='skill/validation/hdl')
    parser.add_argument('--workspace', default='.')
    args = parser.parse_args()
    result = run(args.iverilog, args.vvp, args.directory, args.workspace)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['status']=='pass' else 1)
