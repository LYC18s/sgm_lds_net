#!/usr/bin/env python3
"""Reproducible offline acceptance. All outputs/temp files remain inside workspace."""
import argparse
import ast
from datetime import datetime, timezone
import io
import importlib.util
import json
from pathlib import Path
import re
import sys
import unittest

from axis_core import analyze, compare_binary, decode_dma, digest, load_json, read_trace, render_report, summarize_benchmark
from axis_tool import output_path
from make_examples import generate


def metadata_check(root):
    """Validate this package's deliberately small frontmatter without adding PyYAML."""
    text = (root/'SKILL.md').read_text(encoding='utf-8')
    match = re.match(r'^---\n(.*?)\n---\n', text, re.S)
    if not match:
        raise ValueError('Missing skill frontmatter')
    entries = {}
    for line in match[1].splitlines():
        key, value = line.split(':', 1)
        value = value.strip()
        entries[key] = json.loads(value) if value.startswith('"') else value
    if set(entries) != {'name','description'} or not re.fullmatch('[a-z0-9]+(?:-[a-z0-9]+)*', entries['name']):
        raise ValueError('Invalid skill metadata fields/name')
    if len(entries['name']) > 64 or not 0 < len(entries['description']) <= 1024:
        raise ValueError('Invalid skill metadata lengths')
    missing = []
    for path in root.rglob('*.md'):
        for link in re.findall(r'\]\(([^)]+)\)', path.read_text(encoding='utf-8')):
            if '://' in link or link.startswith('#'):
                continue
            target = (path.parent/link.split('#')[0]).resolve()
            if not target.exists():
                missing.append('%s: %s' % (path.relative_to(root),link))
    if missing:
        raise ValueError('Broken local links: '+', '.join(missing))
    for path in root.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path), feature_version=(3,8))
    return dict(status='pass', frontmatter=entries, python_minimum_syntax='3.8', local_links='pass',
                official_quick_validate='not_run: standard-library acceptance does not invoke an external validator',
                pyyaml_available=bool(importlib.util.find_spec('yaml')),
                note='Standard-library checks cover this package, not arbitrary YAML syntax')


def run(workspace, root):
    out = root/'validation'
    output_path(out/'selftest.json', workspace).write_text('{"status":"running"}\n',encoding='utf-8')
    cases = generate(root/'examples/generated', workspace)
    results, case_records = [], []
    for case in cases:
        conf_path = root/'examples/generated'/(case['name']+'.json')
        trace_path = root/'examples/generated'/(case['name']+'.csv')
        conf = load_json(conf_path)
        result = analyze(read_trace(trace_path,conf),conf)
        result['evidence'] = {str(p.relative_to(root)):digest(p) for p in (conf_path,trace_path)}
        result['evidence_level'] = 'synthetic'
        matches = result['status']==case['expected'] and (not case['rule'] or
                  case['rule'] in [f['rule'] for f in result['failures']])
        case_records.append(dict(name=case['name'], expected=case['expected'], actual=result['status'], accepted=matches))
        output_path(out/(case['name']+'.json'),workspace).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        results.append(result)
    for filename in ('dma_idle.json','dma_decode_error.json'):
        result=decode_dma(load_json(root/'examples/generated'/filename))
        result['evidence_level']='synthetic'
        results.append(result)
        output_path(out/filename,workspace).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for actual in ('actual_ok.bin','actual_bad.bin'):
        result=compare_binary(root/'examples/generated/expected.bin',root/'examples/generated'/actual)
        result['evidence_level']='synthetic'
        results.append(result)
        output_path(out/(actual+'.json'),workspace).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    import csv
    with (root/'examples/generated/timings.csv').open(newline='') as handle:
        result=summarize_benchmark(csv.DictReader(handle),1)
    result['evidence_level']='synthetic'
    results.append(result)
    output_path(out/'benchmark.json',workspace).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    output_path(out/'example_report.md',workspace).write_text(render_report(results),encoding='utf-8')
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(root/'tests'),pattern='test_*.py')
    test_result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    output_path(out/'unittest.log',workspace).write_text(stream.getvalue(),encoding='utf-8')
    metadata=metadata_check(root)
    record=dict(status='pass' if test_result.wasSuccessful() and all(c['accepted'] for c in case_records) else 'fail',
                generated_at_utc=datetime.now(timezone.utc).isoformat(), python=sys.version,
                unittest=dict(run=test_result.testsRun,failures=len(test_result.failures),errors=len(test_result.errors)),
                normal_random_stall_seeds=100, corpus_cases=case_records, metadata=metadata,
                verification_levels={'offline_synthetic':'executed','software_mmio_model':'executed',
                                     'new_hdl_bench':'not_executed','board':'not_executed','cross_board':'not_executed'},
                source_sha256={p.relative_to(root).as_posix():digest(p) for parent in ('scripts','tests','assets','references','agents')
                               for p in sorted((root/parent).rglob('*')) if p.is_file() and '__pycache__' not in p.parts})
    record['source_sha256']['SKILL.md']=digest(root/'SKILL.md')
    record['source_sha256']['README.md']=digest(root/'README.md')
    output_path(out/'selftest.json',workspace).write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',default='.')
    args=parser.parse_args()
    record=run(Path(args.workspace).resolve(),Path(__file__).resolve().parents[1])
    print(json.dumps({key:record[key] for key in ('status','unittest','verification_levels')},ensure_ascii=False,indent=2))
    raise SystemExit(0 if record['status']=='pass' else 1)
