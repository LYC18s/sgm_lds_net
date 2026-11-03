#!/usr/bin/env python3
"""Capture hashes and bounded matching source excerpts for an experience record."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re

from axis_core import digest, load_json
from axis_tool import output_path


def collect(manifest, source_root):
    root=Path(source_root).resolve()
    sources=[]
    for item in manifest['sources']:
        path=(root/item['path']).resolve()
        path.relative_to(root)
        lines=path.read_text(encoding='utf-8-sig',errors='replace').splitlines()
        pattern=re.compile(item['pattern'])
        matches=[]
        for i,line in enumerate(lines):
            if pattern.search(line):
                lo,hi=max(0,i-1),min(len(lines),i+3)
                matches.append(dict(line_start=lo+1,excerpt='\n'.join(lines[lo:hi])))
                if len(matches)>=4:break
        sources.append(dict(path=item['path'],sha256=digest(path),matches=matches,
                            evidence_type=item['evidence_type']))
    return dict(captured_at_utc=datetime.now(timezone.utc).isoformat(),sources=sources,
                limitation='Historical records/source inspection only; these are not reruns or current-board claims')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--source-root',default='.')
    parser.add_argument('--workspace',default='.')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=collect(load_json(args.manifest),args.source_root)
    output_path(args.output,args.workspace).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Captured %d source records without changing originals.' % len(result['sources']))
