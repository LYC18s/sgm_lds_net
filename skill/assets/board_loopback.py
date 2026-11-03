"""Callable board example; importing/running this file alone never starts hardware.

Call run() from the board's already prepared workspace after reviewing ownership,
matching bit/HWH and endpoint behavior. No Overlay download or global installation.
"""
import csv
import json
from pathlib import Path
import sys

SKILL_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SKILL_ROOT/'scripts'))
from axis_core import digest, load_json
from axis_tool import output_path
from board_session import QUARANTINE, TransferFailure, run_simple, validate_parameters


def run(mmio, ip_description, *, workspace, config_path, output_directory,
        exclusive_owner=False, start_ip=None, reference=None):
    """ip_description is the actual loaded Overlay.ip_dict[dma_name] entry.

    MMIO is provided separately to avoid instantiating a DMA driver with side effects.
    Caller verifies it maps that exact IP. reference accepts packed input bytes and
    returns packed expected output bytes. Identity loopback is the default only when
    contract.reference == 'identity'.
    """
    if not exclusive_owner:
        raise ValueError('Stop all existing owners and explicitly grant exclusive ownership')
    cfg=load_json(config_path)
    if cfg['ip_type']!='axi_dma' or cfg['mode']!='simple' or cfg['dtype']!='uint8':
        raise ValueError('This example requires packed uint8 non-Micro AXI DMA Simple')
    if not str(ip_description.get('type','')).startswith('xilinx.com:ip:axi_dma:'):
        raise ValueError('Actual exported IP is not AXI DMA')
    parameters=ip_description['parameters']
    validate_parameters(parameters,('mm2s','s2mm'))
    for key in ('tx_bytes','rx_capacity_bytes','expected_rx_bytes','repeats'):
        if type(cfg[key]) is not int or cfg[key]<=0:
            raise ValueError('Expected positive integer '+key)
    if type(cfg['warmup']) is not int or cfg['warmup']<0:
        raise ValueError('warmup must be a nonnegative integer')
    if cfg['expected_rx_bytes']>cfg['rx_capacity_bytes']:
        raise ValueError('Expected output exceeds capacity')
    if reference is None:
        if cfg['reference']!='identity' or cfg['tx_bytes']!=cfg['expected_rx_bytes']:
            raise ValueError('Supply an independent reference for this endpoint contract')
        reference=lambda payload:payload
    workspace=Path(workspace).resolve()
    # Files may be read outside workspace, but all outputs are checked against it.
    build_files=[(workspace/cfg[key]).resolve() for key in ('bitstream','hwh')]
    build_hashes={str(path):digest(path) for path in build_files}
    target=output_path(Path(output_directory)/'board_results.json',workspace)
    if QUARANTINE:
        raise RuntimeError('Resolve prior quarantined transaction before allocating')
    # Runtime dependencies are imported only after explicit invocation and preflight.
    import numpy as np
    from pynq import allocate
    tx=rx=None
    records=[]
    try:
        tx=allocate(shape=(cfg['tx_bytes'],),dtype=np.uint8)
        rx=allocate(shape=(cfg['rx_capacity_bytes'],),dtype=np.uint8)
        for iteration in range(cfg['warmup']+cfg['repeats']):
            tx[:]=(np.arange(cfg['tx_bytes'],dtype=np.uint64)+iteration*17).astype(np.uint8)
            rx[:]=0xa5
            expected=bytes(reference(tx.tobytes()))
            if len(expected)!=cfg['expected_rx_bytes']:
                raise ValueError('Reference output byte count differs from contract')
            result=run_simple(mmio,parameters,tx=tx,rx=rx,tx_bytes=cfg['tx_bytes'],
                              rx_bytes=cfg['rx_capacity_bytes'],expected_rx_bytes=cfg['expected_rx_bytes'],
                              timeout_s=cfg['timeout_s'],poll_s=cfg['poll_s'],start_ip=start_ip,
                              exclusive_owner=True,ip_type='axi_dma')
            actual=rx[:result['rx_actual_bytes']].tobytes()
            output_path(Path(output_directory)/'expected.bin',workspace).write_bytes(expected)
            output_path(Path(output_directory)/'actual.bin',workspace).write_bytes(actual)
            result.update(iteration=iteration,warmup=iteration<cfg['warmup'],data_matches=actual==expected)
            if actual!=expected or result['status']!='pass':
                result['status']='fail'
                records.append(result)
                raise RuntimeError('Output mismatch; inspect saved bytes and transfer evidence')
            records.append(result)
    except BaseException as exc:
        if isinstance(exc,TransferFailure):
            records.append(exc.evidence)
        target.write_text(json.dumps(dict(status='fail',reason=str(exc),build_sha256=build_hashes,records=records),indent=2)+'\n',encoding='utf-8')
        raise
    finally:
        retained=[buffer for entry in QUARANTINE for buffer in entry['buffers']]
        for buffer in (tx,rx):
            if buffer is not None and not any(buffer is held for held in retained):
                buffer.freebuffer()
    target.write_text(json.dumps(dict(status='pass',build_sha256=build_hashes,records=records),indent=2)+'\n',encoding='utf-8')
    with output_path(Path(output_directory)/'timings.csv',workspace).open('w',encoding='utf-8',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=('scope','bytes','seconds'))
        writer.writeheader()
        for result in records:
            writer.writerow(dict(scope=result['scope'],bytes=result['rx_actual_bytes'],seconds=result['seconds']))
    return records
