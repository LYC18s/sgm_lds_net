"""Behavioral regressions: standards cases, evidence limits and DMA ownership failures."""
import copy
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from axis_core import analyze, compare_binary, contract, decode_dma, extract_payload, read_trace, summarize_benchmark
from axis_tool import convert_vcd, main, output_path, inspect_hwh
import board_session as board
from make_examples import generate, packet_rows, video_rows


def config():
    return json.loads((ROOT/'assets/packet.json').read_text(encoding='utf-8'))


def sample(cycle, **changes):
    row = dict(cycle=cycle, resetn=1, valid=1, ready=1, data=123, keep=15, last=0, user=0)
    row.update(changes)
    return row


class TraceTests(unittest.TestCase):
    def test_normal_random_stalls_count_only_handshakes(self):
        result = analyze(packet_rows(), config())
        self.assertEqual(result['status'], 'pass')
        self.assertEqual(result['counts']['handshakes'], 6)
        self.assertEqual(result['counts']['valid_bytes'], 20)
        self.assertEqual([p['valid_bytes'] for p in result['packets']], [10, 10])
        self.assertGreater(result['counts']['backpressure_cycles'], 0)

    def test_many_seeds_no_false_positive(self):
        for seed in range(100):
            with self.subTest(seed=seed):
                self.assertEqual(analyze(packet_rows(seed), config())['status'], 'pass')

    def test_changes_on_accepting_cycle_are_still_violations(self):
        conf = config()
        conf['stream'].pop('packet_bytes')
        conf['stream'].pop('expected_packets')
        result = analyze([sample(0, ready=0), sample(1, data=124, last=1)], conf)
        self.assertIn('STALL_STABILITY', [r['rule'] for r in result['failures']])

    def test_tlast_and_keep_stable_under_stall(self):
        for key, value in [('last', 1), ('keep', 3), ('user', 1)]:
            with self.subTest(key=key):
                result = analyze([sample(0, ready=0), sample(1, **{key:value})], config())
                self.assertIn('STALL_STABILITY', [r['rule'] for r in result['failures']])

    def test_null_lane_data_can_change(self):
        conf = config()
        conf['stream'].update(packet_bytes=2, expected_packets=1)
        result = analyze([sample(0, ready=0, keep=3, data=0x1234, last=1),
                          sample(1, keep=3, data=0xabcd1234, last=1)], conf)
        self.assertEqual(result['status'], 'pass')

    def test_position_byte_not_compared_but_reserved_strb_fails(self):
        conf = config()
        conf['stream'].update(packet_bytes=4, expected_packets=1)
        result = analyze([sample(0, ready=0, strb=0, data=0, last=1),
                          sample(1, strb=0, data=999, last=1)], conf)
        self.assertEqual(result['status'], 'pass')
        result = analyze([sample(0, keep=1, strb=15, last=1)], conf)
        self.assertIn('STRB_WITHOUT_KEEP', [r['rule'] for r in result['failures']])

    def test_legal_long_stall_has_no_protocol_failure(self):
        conf = config()
        conf['capture']['complete_end'] = False
        result = analyze([sample(i, ready=0) for i in range(100)], conf)
        self.assertEqual(result['failures'], [])
        self.assertEqual(result['status'], 'inconclusive')

    def test_application_deadline_is_separate(self):
        conf = config()
        conf['stream']['max_no_handshake_cycles'] = 2
        result = analyze([sample(i, ready=0) for i in range(5)], conf)
        self.assertIn('APPLICATION_NO_PROGRESS', [r['rule'] for r in result['failures']])

    def test_sparse_capture_does_not_compare_across_gap(self):
        conf = config()
        conf['capture'].update(contiguous=False, complete_end=False)
        result = analyze([sample(0, ready=0), sample(12, data=999)], conf)
        self.assertEqual(result['failures'], [])
        self.assertIsNone(result['metrics']['window_bandwidth_Bps'])
        self.assertIsNone(result['metrics']['handshake_utilization'])
        self.assertEqual(result['status'], 'inconclusive')

    def test_declared_contiguous_gap_is_input_error(self):
        with self.assertRaises(ValueError):
            analyze([sample(0), sample(2)], config())

    def test_reset_aborts_pending_stability_obligation(self):
        conf = config()
        conf['stream'].update(packet_bytes=4, expected_packets=1)
        result = analyze([sample(0, ready=0), sample(1, resetn=0, valid=0),
                          sample(2, data=999, last=1)], conf)
        self.assertEqual(result['status'], 'pass')

    def test_video_line_and_frame_semantics(self):
        conf = json.loads((ROOT/'assets/video.json').read_text(encoding='utf-8'))
        result = analyze(video_rows(), conf)
        self.assertEqual(result['status'], 'pass')
        self.assertEqual(result['counts']['completed_frames'], 2)
        rows = video_rows()
        rows[5]['user'] = rows[6]['user'] = 1
        result = analyze(rows, conf)
        self.assertIn('EARLY_SOF', [r['rule'] for r in result['failures']])

    def test_128bit_stream_and_nontrivial_keep(self):
        conf = config()
        conf['stream'].update(data_width_bits=128, packet_bytes=8, expected_packets=1)
        row = sample(0, keep=0x5555, data=(1<<127)+1, last=1)
        result = analyze([row], conf)
        self.assertEqual(result['status'], 'pass')
        self.assertEqual(result['counts']['valid_bytes'], 8)

    def test_no_traffic_is_not_validated_transfer(self):
        conf = config()
        conf['stream'].pop('expected_packets')
        self.assertEqual(analyze([sample(0, valid=0)], conf)['status'], 'inconclusive')

    def test_interleaving_refused(self):
        conf = config()
        conf['signals']['id'] = 'tid'
        conf['stream']['interleaved'] = True
        with self.assertRaises(ValueError):
            contract(conf)

    def test_invalid_boolean_control_refused(self):
        with self.assertRaises(ValueError):
            analyze([sample(0, ready=2)], config())


class FilesTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'validation/tmp').mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT/'validation/tmp')
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_generated_corpus(self):
        cases = generate(self.directory, ROOT)
        for case in cases:
            with self.subTest(case=case['name']):
                conf = json.loads((self.directory/(case['name']+'.json')).read_text())
                result = analyze(read_trace(self.directory/(case['name']+'.csv'), conf), conf)
                self.assertEqual(result['status'], case['expected'])
                if case['rule']:
                    self.assertIn(case['rule'], [r['rule'] for r in result['failures']])

    def test_payload_compare_first_mismatch_and_size(self):
        a,b = self.directory/'a.bin', self.directory/'b.bin'
        a.write_bytes(b'012345')
        b.write_bytes(b'012')
        self.assertEqual(compare_binary(a,b)['first_mismatch_offset'], 3)
        b.write_bytes(b'019345')
        self.assertEqual(compare_binary(a,b)['first_mismatch_offset'], 2)
        b.write_bytes(a.read_bytes())
        self.assertEqual(compare_binary(a,b)['status'], 'pass')

    def test_extracted_handshake_bytes_match_golden(self):
        generate(self.directory,ROOT)
        cfg=config()
        actual=extract_payload(read_trace(self.directory/'packet_ok.csv',cfg),32)
        self.assertEqual(actual,(self.directory/'expected.bin').read_bytes())

    def test_vcd_dumpoff_is_not_misreported_as_continuous(self):
        source=self.directory/'gapped.vcd'
        source.write_text('$scope module t $end\n$var wire 1 ! clk $end\n$upscope $end\n$enddefinitions $end\n#0\n0!\n#5\n1!\n$dumpoff\n$end\n')
        with self.assertRaises(ValueError):
            convert_vcd(source,{'clock':'t.clk'},self.directory/'bad.csv')

    def test_workspace_escape_refused_without_creating_target(self):
        with self.assertRaises(ValueError):
            output_path(ROOT.parent/'forbidden-skill-output.json', ROOT)

    def test_cli_exit_codes(self):
        generate(self.directory, ROOT)
        with patch('sys.stdout', new=io.StringIO()), patch('sys.stderr', new=io.StringIO()):
            for name, code in [('packet_ok',0), ('missing_tlast',1), ('truncated_capture',3)]:
                self.assertEqual(main(['check', str(self.directory/(name+'.csv')), '--config',
                                       str(self.directory/(name+'.json'))]), code)
            self.assertEqual(main(['check', str(self.directory/'missing.csv'), '--config',
                                   str(self.directory/'packet_ok.json')]), 2)

    def test_cli_refuses_to_overwrite_input(self):
        generate(self.directory,ROOT)
        source=self.directory/'packet_ok.csv'
        original=source.read_bytes()
        with patch('sys.stderr',new=io.StringIO()):
            code=main(['--output',str(source),'check',str(source),'--config',str(self.directory/'packet_ok.json')])
        self.assertEqual(code,2)
        self.assertEqual(source.read_bytes(),original)

    def test_unknown_payload_ignored_only_when_invalid(self):
        path = self.directory/'unknown.csv'
        header = 'cycle,resetn,valid,ready,data,keep,last,user\n'
        path.write_text(header+'0,1,0,1,x,x,x,x\n', encoding='utf-8')
        self.assertEqual(list(read_trace(path, config()))[0]['data'], 0)
        path.write_text(header+'0,1,1,1,x,15,1,0\n', encoding='utf-8')
        with self.assertRaises(ValueError):
            list(read_trace(path, config()))

    def test_vcd_preedge_sampling(self):
        path = self.directory/'a.vcd'
        path.write_text('''$scope module tb $end
$var wire 1 ! clk $end
$var wire 1 " valid $end
$var wire 8 # data $end
$upscope $end
$enddefinitions $end
#0
0!
1"
b00101010 #
#5
1!
b00000001 #
#10
0!
#15
1!
''', encoding='utf-8')
        target = self.directory/'a.csv'
        convert_vcd(path, {'clock':'tb.clk','valid':'tb.valid','data':'tb.data'}, target)
        with target.open() as handle:
            rows=list(csv.DictReader(handle))
        self.assertEqual([int(r['data'],2) for r in rows], [42,1])

    def test_hwh_inventory_does_not_confuse_vdma(self):
        path = self.directory/'x.hwh'
        path.write_text('<ROOT><MODULE FULLNAME="/dma" VLNV="xilinx.com:ip:axi_dma:7.1"><PARAMETERS><PARAMETER NAME="C_INCLUDE_SG" VALUE="0"/></PARAMETERS></MODULE></ROOT>')
        self.assertEqual(inspect_hwh(path)['modules'][0]['parameters']['C_INCLUDE_SG'],'0')
        path.write_text('<!DOCTYPE a><ROOT/>')
        with self.assertRaises(ValueError):
            inspect_hwh(path)


class MetricsTests(unittest.TestCase):
    def test_warmup_percentiles_and_scope_separation(self):
        rows=[dict(scope='python_e2e',bytes=100,seconds=x) for x in (99,1,2,3)]
        result=summarize_benchmark(rows,1)['groups'][0]
        self.assertEqual(result['median_s'],2)
        self.assertAlmostEqual(result['p95_s'],2.9)
        self.assertEqual(result['aggregate_Bps'],50)
        rows.append(dict(scope='hardware',bytes=100,seconds=.1))
        self.assertEqual(len(summarize_benchmark(rows,1)['groups']),2)

    def test_invalid_timing_refused(self):
        for duration in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                summarize_benchmark([dict(scope='dma',bytes=10,seconds=duration)])

    def test_register_decode_and_vdma_rejection(self):
        result=decode_dma(dict(ip_type='axi_dma',registers={'S2MM_DMASR':'0x4041'}))
        self.assertEqual(result['channels']['S2MM']['errors'], ['DMADecErr'])
        self.assertEqual(result['status'],'fail')
        result=decode_dma(dict(ip_type='axi_dma',registers={'S2MM_DMASR':'0x1002'}))
        self.assertEqual(result['status'],'inconclusive')
        with self.assertRaises(ValueError):
            decode_dma(dict(ip_type='axi_vdma', registers={'S2MM_DMASR':0}))


PARAMETERS = dict(C_INCLUDE_SG=0,C_MICRO_DMA=0,C_SG_LENGTH_WIDTH=14,C_ADDR_WIDTH=32,
                  C_INCLUDE_MM2S=1,C_INCLUDE_S2MM=1,C_INCLUDE_MM2S_DRE=0,C_INCLUDE_S2MM_DRE=0,
                  C_M_AXI_MM2S_DATA_WIDTH=32,C_M_AXI_S2MM_DATA_WIDTH=32)


class Buffer:
    def __init__(self,address,size=16):
        self.device_address=address
        self.nbytes=size
        self.data=bytearray(size)
        self.operations=[]
        self.coherent=False
    def flush(self):self.operations.append('flush')
    def invalidate(self):self.operations.append('invalidate')


class MMIO:
    """Stateful hardware model: LENGTH starts a transaction; IOC only at completion."""
    def __init__(self,tx,rx,hang=False,error=False):
        self.reg={0:0,4:1,0x30:0,0x34:1}
        self.writes=[]
        self.tx,self.rx=tx,rx
        self.hang,self.error=hang,error
        self.launched=set()
        self.polls=0
    def read(self,offset):
        if offset in (4,0x34) and self.launched=={0,0x30}:
            self.polls+=1
            if self.error:self.reg[0x34]=0x4041
            elif not self.hang and self.polls>=4:
                self.rx.data[:]=self.tx.data
                self.reg[4]=self.reg[0x34]=0x1002
                self.reg[0x58]=len(self.tx.data)
        return self.reg.get(offset,0)
    def write(self,offset,value):
        self.writes.append((offset,value))
        if offset in (4,0x34):self.reg[offset]&=~value
        else:self.reg[offset]=value
        if offset in (0,0x30):self.reg[offset+4]=0 if value&1 else 1
        if offset in (0x28,0x58):
            base=offset-0x28
            self.reg[base+4]=0
            self.launched.add(base)


class BoardTests(unittest.TestCase):
    def setUp(self):
        board.QUARANTINE.clear()
        self.tx,self.rx=Buffer(0x10000),Buffer(0x20000)
        self.tx.data[:]=bytes(range(16))
        self.mmio=MMIO(self.tx,self.rx)
    def tearDown(self):
        # Mock buffers only. Real recovery is never simulated by clearing this list.
        board.QUARANTINE.clear()
    def run_transfer(self,**kwargs):
        return board.run_simple(self.mmio,PARAMETERS,tx=self.tx,rx=self.rx,
                                exclusive_owner=True,ip_type='axi_dma',timeout_s=.02,poll_s=.0001,**kwargs)
    def test_rx_first_completion_cache_and_data(self):
        result=self.run_transfer(expected_rx_bytes=16)
        self.assertEqual(result['status'],'pass')
        self.assertEqual(self.tx.data,self.rx.data)
        self.assertLess([o for o,v in self.mmio.writes].index(0x58),[o for o,v in self.mmio.writes].index(0x28))
        self.assertEqual(self.rx.operations,['flush','invalidate'])
        self.assertEqual(self.tx.operations,['flush'])
        self.assertFalse(board.QUARANTINE)
    def test_wrong_length_is_functional_failure_after_completion(self):
        self.assertEqual(self.run_transfer(expected_rx_bytes=15)['status'],'fail')
        self.assertFalse(board.QUARANTINE)
    def test_timeout_preserves_buffers_and_prevents_retry(self):
        self.mmio.hang=True
        with self.assertRaises(board.TransferFailure) as ctx:self.run_transfer()
        self.assertTrue(ctx.exception.evidence['buffers_retained'])
        self.assertIs(board.QUARANTINE[0]['buffers'][0],self.rx)
        with self.assertRaises(RuntimeError):self.run_transfer()
        with self.assertRaises(RuntimeError):board.release_quarantine_after_recovery(0,upstream_quiescent=True)
        self.mmio.launched.clear()
        self.mmio.write(0,0);self.mmio.write(0x30,0)
        held=board.release_quarantine_after_recovery(0,upstream_quiescent=True)
        self.assertIs(held['buffers'][1],self.tx)
    def test_error_retains_evidence(self):
        self.mmio.error=True
        with self.assertRaises(board.TransferFailure) as ctx:self.run_transfer()
        self.assertEqual(ctx.exception.evidence['after']['registers']['S2MM_DMASR'],'0x00004041')
    def test_keyboard_interrupt_does_not_free_buffers(self):
        with self.assertRaises(KeyboardInterrupt):
            self.run_transfer(start_ip=lambda:(_ for _ in ()).throw(KeyboardInterrupt()))
        self.assertEqual(len(board.QUARANTINE),1)
    def test_invalid_alignment_length_and_overlap_before_writes(self):
        for address,length in [(0x10001,16),(0x10000,0),(0x10000,17)]:
            self.tx.device_address=address
            with self.assertRaises(ValueError):self.run_transfer(tx_bytes=length)
            self.assertEqual(self.mmio.writes,[])
        self.tx.device_address=self.rx.device_address
        with self.assertRaises(ValueError):self.run_transfer()
    def test_sg_and_missing_parameters_refused(self):
        with self.assertRaises(ValueError):board.validate_parameters(dict(PARAMETERS,C_INCLUDE_SG=1),('mm2s',))
        with self.assertRaises(ValueError):board.validate_parameters({},('mm2s',))
    def test_readonly_snapshot_never_writes(self):
        board.snapshot(self.mmio)
        self.assertEqual(self.mmio.writes,[])


if __name__=='__main__':
    unittest.main()
