#!/usr/bin/env python3
"""Copy only this skill into an isolated workspace and run its offline acceptance."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from axis_core import digest
from axis_tool import output_path


def check(root,workspace):
    root=Path(root).resolve()
    temporary_parent=output_path(root/'validation/portability_tmp/reserved',workspace).parent
    with tempfile.TemporaryDirectory(dir=temporary_parent) as directory:
        temporary=Path(directory).resolve()
        temporary.relative_to(Path(workspace).resolve())  # Verified before automatic recursive cleanup.
        copied=temporary/'skill'
        shutil.copytree(root,copied,ignore=shutil.ignore_patterns('validation','__pycache__','*.pyc'))
        (copied/'validation').mkdir()
        env=os.environ.copy()
        cache=temporary/'tmp'
        cache.mkdir()
        env.update(TEMP=str(cache),TMP=str(cache),TMPDIR=str(cache),PYTHONDONTWRITEBYTECODE='1',PYTHONUTF8='1')
        command=[sys.executable,'-B','-X','utf8',str(copied/'scripts/selftest.py'),'--workspace',str(temporary)]
        result=subprocess.run(command,cwd=temporary,env=env,capture_output=True,text=True,encoding='utf-8',timeout=60)
        log=result.stdout+'\n'+result.stderr
        record=load_result= None
        if (copied/'validation/selftest.json').exists():
            load_result=json.loads((copied/'validation/selftest.json').read_text(encoding='utf-8'))
        record=dict(status='pass' if result.returncode==0 and load_result and load_result.get('status')=='pass' else 'fail',
                    isolated_copy=True,original_project_modules_copied=False,exit_code=result.returncode,
                    unittest=load_result.get('unittest') if load_result else None,
                    source_sha256={p.relative_to(root).as_posix():digest(p) for sub in ('scripts','tests','assets','references')
                                   for p in sorted((root/sub).rglob('*')) if p.is_file() and '__pycache__' not in p.parts},
                    limitation='Directory-level offline portability only, not cross-board hardware compatibility')
        output_path(root/'validation/portability.log',workspace).write_text(log,encoding='utf-8')
    output_path(root/'validation/portability.json',workspace).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',default='.')
    args=parser.parse_args()
    result=check(Path(__file__).resolve().parents[1],Path(args.workspace).resolve())
    print(json.dumps({key:result[key] for key in ('status','isolated_copy','unittest','limitation')},indent=2))
    raise SystemExit(0 if result['status']=='pass' else 1)
