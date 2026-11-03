"""Content-based module-reference cache ownership, including nested RTL."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'build/lds_net_system'
STATE=BUILD/'ooc_sources.json'
PLAN=BUILD/'ooc_pending.json'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def inputs():
    modules={};texts={}
    for path in (ROOT/'hardware/rtl').rglob('*.v'):
        text=re.sub(r'/\*.*?\*/|//[^\n]*','',path.read_text(errors='replace'),flags=re.S)
        texts[path]=text
        for name in re.findall(r'\bmodule\s+(\w+)',text):
            if name in modules and modules[name]!=path:raise RuntimeError('Ambiguous RTL module '+name)
            modules[name]=path
    graph=json.loads((BUILD/'lds_net_system.srcs/sources_1/bd/system/system.bd').read_text())
    result={}
    def walk(cells):
        for cell in cells.values():
            if ':module_ref:' in cell.get('vlnv',''):
                reference=cell['reference_info']['ref_name'];ip=cell['xci_name']
                if not re.fullmatch(r'\w+',ip):raise RuntimeError('Unsafe module IP name')
                todo=[reference];seen=set();paths=set()
                while todo:
                    name=todo.pop()
                    if name in seen:continue
                    seen.add(name)
                    if name not in modules:raise RuntimeError('Missing module-reference source '+name)
                    path=modules[name];paths.add(path)
                    tokens=set(re.findall(r'\b[A-Za-z_]\w*\b',texts[path]))
                    todo.extend(tokens.intersection(modules).difference(seen))
                result[ip]=dict(reference=reference,
                    parameters={k:v.get('value') for k,v in cell.get('parameters',{}).items()},
                    sources={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(paths)})
            walk(cell.get('components',{}))
    walk(graph['design']['components'])
    if not any(v['reference']=='sgm_disparity_dma' for v in result.values()):
        raise RuntimeError('Transactional SGM DMA missing from module-reference graph')
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=('plan','record','check'))
    mode=parser.parse_args().mode;current=inputs()
    old=json.loads(STATE.read_text()) if STATE.exists() else {}
    if mode=='plan':
        # Recover successful sibling leaves after an interrupted/failed build.
        # Their inputs must match the pre-launch plan, with a new checkpoint
        # and Vivado's successful completion marker; timestamps alone do not
        # certify a cache originating before this content-based tracking.
        pending=json.loads(PLAN.read_text()) if PLAN.exists() else None
        if pending:
            for ip in pending['dirty']:
                folder=BUILD/'lds_net_system.runs'/(ip+'_synth_1')
                dcp=folder/(ip+'.dcp');log=folder/(ip+'.vds')
                if (current.get(ip)==pending['inputs'].get(ip) and dcp.exists()
                        and dcp.stat().st_mtime>=pending['started']-1
                        and (folder/'.vivado.end.rst').exists()
                        and (folder/'.vivado.end.rst').stat().st_mtime>=dcp.stat().st_mtime-1 and log.exists()
                        and 'ERROR:' not in log.read_text(errors='replace')):
                    old[ip]={**current[ip],'dcp_sha256':sha(dcp)}
            STATE.write_text(json.dumps(old,indent=2)+'\n')
        dirty=[]
        for ip,entry in current.items():
            dcp=BUILD/'lds_net_system.runs'/(ip+'_synth_1')/(ip+'.dcp')
            saved=old.get(ip,{})
            if (any(saved.get(k)!=v for k,v in entry.items()) or not dcp.exists()
                    or saved.get('dcp_sha256')!=sha(dcp)):
                dirty.append(ip)
        PLAN.write_text(json.dumps(dict(started=time.time(),inputs=current,dirty=dirty),indent=2)+'\n')
        print(' '.join(sorted(dirty)))
        return
    if mode=='record':
        plan=json.loads(PLAN.read_text())
        if current!=plan['inputs']:raise RuntimeError('Module-reference inputs changed during synthesis')
        for ip,entry in current.items():
            dcp=BUILD/'lds_net_system.runs'/(ip+'_synth_1')/(ip+'.dcp')
            if ip in plan['dirty'] and dcp.stat().st_mtime<plan['started']-1:
                raise RuntimeError('Old checkpoint reused after forced refresh: '+ip)
            entry['dcp_sha256']=sha(dcp)
        STATE.write_text(json.dumps(current,indent=2)+'\n')
    else:
        for ip,entry in current.items():
            dcp=BUILD/'lds_net_system.runs'/(ip+'_synth_1')/(ip+'.dcp')
            saved=old.get(ip,{})
            if any(saved.get(k)!=v for k,v in entry.items()) or saved.get('dcp_sha256')!=sha(dcp):
                raise RuntimeError('Stale module-reference cache '+ip)
    print('PASS: every module-reference checkpoint matches its complete current RTL dependency hashes')


if __name__=='__main__':main()
