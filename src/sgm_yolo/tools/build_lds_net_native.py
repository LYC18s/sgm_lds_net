"""Build the small C++11 CPU decoder; never invokes a model framework."""
import argparse
from pathlib import Path
import os
import hashlib
import json
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--target',choices=('host','armhf'),default='host')
    args = parser.parse_args()
    compiler = args.compiler or os.environ.get('ARM_CXX' if args.target=='armhf' else 'CXX') or (shutil.which('arm-linux-gnueabihf-g++') if args.target=='armhf' else shutil.which('g++'))
    if not compiler:
        raise RuntimeError('C++11 compiler missing; set CXX or --compiler')
    destination = args.output or ROOT / ('build/lds_net/armhf/liblds_net_post.so' if args.target=='armhf' else 'build/lds_net/lds_net_post.dll' if os.name == 'nt' else 'dual_ov5640/liblds_net_post.so')
    destination.parent.mkdir(parents=True, exist_ok=True)
    command = [compiler, '-std=c++11', '-O3', '-Wall', '-Wextra', '-Werror', '-fno-exceptions', '-fno-rtti', '-shared']
    if args.target=='armhf':
        command += ['-mcpu=cortex-a9','-mfpu=neon','-mfloat-abi=hard','-fPIC','-static-libstdc++','-static-libgcc','-Wl,--no-undefined','-Wl,-soname,liblds_net_post.so']
    elif os.name == 'nt':
        command += ['-static-libgcc', '-static-libstdc++', '-Wl,-Bstatic', '-lwinpthread']
    else:
        command += ['-fPIC']
    command += [str(ROOT / 'native/lds_net_post.cpp'), '-o', str(destination)]
    subprocess.run(command, check=True)
    if args.target=='armhf':
        readelf=Path(compiler).with_name(Path(compiler).name.replace('g++','readelf'))
        details=subprocess.check_output([str(readelf),'-h','-d','-V',str(destination)],text=True,encoding='utf-8')
        versions=sorted({tuple(map(int,v.split('.'))) for v in re.findall(r'GLIBC_([0-9.]+)',details)})
        if 'ELF32' not in details or 'Machine:                           ARM' not in details or 'hard-float ABI' not in details:
            raise RuntimeError('Native library is not ARM32 hard-float ELF')
        if 'GLIBCXX_' in details or 'libstdc++.so' in details or not versions or versions[-1]>(2,27):
            raise RuntimeError('Native library exceeds the supported PYNQ glibc2.27 baseline')
        digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
        report=dict(format='lds_net.native.v1',target='armv7-cortex-a9-linux-gnueabihf',abi='0x524c0001',
            tested_on_target=False,source_sha256=digest(ROOT/'native/lds_net_post.cpp'),library_sha256=digest(destination),
            minimum_glibc='.'.join(map(str,versions[-1])),compiler=str(compiler),command=command)
        destination.with_suffix('.build.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        destination.with_suffix('.readelf.txt').write_text(details,encoding='utf-8')
    print('PASS: native decoder compiled: ' + str(destination))


if __name__ == '__main__':
    main()
