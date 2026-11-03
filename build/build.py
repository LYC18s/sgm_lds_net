"""Build the current design with explicit tools and repository-local outputs."""
import argparse
import datetime
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'src/sgm_yolo'


def short_root(root):
    """Keep the actual source path; do not create drive mappings."""
    return Path(root).resolve(), False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True, choices=['project', 'synth', 'route', 'export', 'native'])
    parser.add_argument('--vivado', help='Existing vivado or vivado.bat executable')
    parser.add_argument('--compiler', help='Existing C++ compiler, or command available in PATH')
    parser.add_argument('--target', choices=['host', 'armhf'], default='host')
    args = parser.parse_args()
    if args.stage != 'native' and not args.vivado:
        parser.error('--vivado is required for FPGA stages')
    if args.stage == 'native' and not args.compiler:
        parser.error('--compiler is required for native compilation')
    # Resolve tools before creating any build output.
    selected = args.compiler if args.stage == 'native' else args.vivado
    tool = shutil.which(selected) or (str(Path(selected).resolve()) if Path(selected).is_file() else None)
    if not tool:
        parser.error('Tool does not exist: ' + selected)
    work = ROOT / 'build/runtime'
    work.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    for key, directory in [('TEMP', 'tmp'), ('TMP', 'tmp'), ('APPDATA', 'profile'), ('LOCALAPPDATA', 'local'), ('XDG_CACHE_HOME', 'cache'), ('MPLCONFIGDIR', 'mpl'), ('NUMBA_CACHE_DIR', 'numba')]:
        target = work / directory
        target.mkdir(exist_ok=True)
        env[key] = str(target)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONUTF8'] = '1'
    env['LDS_NET_BUILD_PYTHON'] = sys.executable
    (PROJECT / 'build').mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log = work / f'{args.stage}_{stamp}.log'
    if args.stage == 'native':
        command = [sys.executable, '-B', str(PROJECT / 'tools/build_lds_net_native.py'), '--compiler', tool, '--target', args.target]
        if args.target == 'armhf' or os.name != 'nt':
            command += ['--output', str(PROJECT / 'dual_ov5640/liblds_net_post.so')]
    else:
        script = PROJECT / 'tools' / ('export_lds_net_board.tcl' if args.stage == 'export' else 'build_lds_net_system.tcl')
        command = [tool, '-mode', 'batch', '-notrace', '-source', str(script), '-log', str(work / f'{args.stage}_{stamp}.vivado.log'), '-journal', str(work / f'{args.stage}_{stamp}.jou')]
        if args.stage != 'export':
            command += ['-tclargs', 'bd' if args.stage == 'project' else args.stage]
    actual = command
    if os.name == 'nt' and str(command[0]).lower().endswith(('.bat', '.cmd')):
        if any(any(c in item for c in '\r\n"%') for item in command):
            raise ValueError('Unsupported batch argument characters')
        actual = subprocess.list2cmdline([env.get('COMSPEC', 'cmd.exe')]) + ' /d /v:off /s /c "' + ' '.join('"' + item + '"' for item in command) + '"'
    print('Stage:', args.stage, '\nLog:', log, flush=True)
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run(actual, cwd=PROJECT, env=env, stdout=stream, stderr=subprocess.STDOUT)
    text = log.read_text(encoding='utf-8', errors='replace')
    if result.returncode or 'ERROR:' in text or 'Fatal:' in text:
        print(text[-5000:])
        raise SystemExit(result.returncode or 1)
    if args.stage != 'native':
        marker = 'PASS: routed LDS-Net bit/HWH exported privately;' if args.stage == 'export' else 'PASS: LDS-Net private system ' + ('bd' if args.stage == 'project' else args.stage) + ' stage completed'
        if marker not in text:
            raise SystemExit('Missing Vivado completion marker; inspect ' + str(log))
    print('Completed:', args.stage)


if __name__ == '__main__':
    main()
