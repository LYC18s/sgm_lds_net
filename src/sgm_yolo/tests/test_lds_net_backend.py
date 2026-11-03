"""PS ownership/cache protocol tests using a stateful MMIO/CMA model.

These do not substitute for the real Verilog perception/AXI tests.
"""
import json
from pathlib import Path
import threading
import time
import unittest
import numpy as np
from dual_ov5640.lds_net_backend import LDSNetPerception,HARDWARE_ABI,DISPLAY_ABI,SIZES

ROOT=Path(__file__).resolve().parents[1]


class Buffer(np.ndarray):
    def __new__(cls,size,address,fabric):
        value=np.zeros(size,np.uint8).view(cls);value.device_address=address;value.fabric=fabric;value.freed=False;return value
    def __array_finalize__(self,parent):
        if parent is not None:
            self.device_address=getattr(parent,'device_address',0);self.fabric=getattr(parent,'fabric',None);self.freed=False
    def flush(self):self.fabric.cache.append(('flush',self.device_address,self.nbytes))
    def invalidate(self):
        self.fabric.cache.append(('invalidate',self.device_address,self.nbytes))
        if self.nbytes==57600 and self.fabric.host is None:raise AssertionError('Mask copied without HOST ownership')
    def freebuffer(self):self.freed=True


class Fabric:
    def __init__(self):
        self.enabled=False;self.host=None;self.result=None;self.active=None;self.pending=False;self.clear_busy=False;self.hang_clear=False
        self.registers={};self.display_registers={};self.buffers=[];self.cache=[];self.decoded=[];self.publications=[]
        self.sha=int(json.loads((ROOT/'models/lds_net/contract.json').read_text())['network_sha256'],16)
        self.nn=MMIO(self,False);self.display=MMIO(self,True)
    def allocate(self,size):
        buffer=Buffer(size,0x1000000+len(self.buffers)*0x200000,self);self.buffers.append(buffer);return buffer
    def decode(self,packet,threshold):
        if self.host is None:raise AssertionError('Detection copied without HOST ownership')
        self.decoded.append((packet.nbytes,packet.dtype,threshold))
        return [dict(x1=79.6,y1=-2.,x2=560.,y2=485.,score=.75,category=0,anchor=0,label='person')]
    def inject(self,slot,frame):
        if self.result is not None or self.host is not None:raise AssertionError('Result token still occupied')
        if self.active is not None and self.active[0]==slot:raise AssertionError('Attempt to reuse displayed mask')
        self.buffers[4+slot][:7000]=np.arange(7000,dtype=np.uint16).astype(np.uint8)
        self.buffers[6+slot][:57600]=0x24
        self.result=(slot,frame)


class MMIO:
    def __init__(self,fabric,display):self.f=fabric;self.is_display=display
    def read(self,address):
        f=self.f
        if self.is_display:
            if address==0:return DISPLAY_ABI
            if address==4:return int(f.pending)|int(f.clear_busy)*2|int(f.active is not None)*32
            if address==8:return 0 if f.active is None else f.active[1]
            if address==12:return 0 if f.active is None else f.active[0]
            return f.display_registers.get(address,0)
        if address==0:return HARDWARE_ABI
        if address==4:
            idle=not (f.enabled or f.host or f.result or f.active or f.pending or f.clear_busy)
            return int(f.enabled)|int(idle)*4|int(f.result is not None and f.host is None)*8|int(f.host is not None)*16|int(f.active is not None)*32
        if address==12:return 31
        if 0x80<=address<=0x9c:return (f.sha>>((address-0x80)*8))&0xffffffff
        if address in (0xa0,0xa4,0xa8,0xac):return {0xa0:307200,0xa4:614400,0xa8:26200,0xac:57600}[address]
        if address in (0x60,0x64,0x68,0x6c) and f.host is not None:
            slot,frame=f.host
            return {0x60:frame,0x64:slot,0x68:f.buffers[4+slot].device_address,0x6c:f.buffers[6+slot].device_address}[address]
        return f.registers.get(address,0)
    def write(self,address,value):
        f=self.f
        if self.is_display:
            if address==4:
                if value&2:
                    f.pending=False
                    if f.hang_clear:f.clear_busy=True
                    else:f.active=None;f.clear_busy=False
                if value&1:
                    slot=f.display_registers[28];frame=f.display_registers[20];base=f.display_registers[24]
                    if f.host!=(slot,frame) or base!=f.buffers[6+slot].device_address:raise AssertionError('Display not authorized by HOST')
                    f.publications.append((slot,frame));f.host=None;f.active=(slot,frame)
            else:
                if f.pending or f.clear_busy:raise AssertionError('Modified frozen display shadow')
                f.display_registers[address]=value
            return
        if address==4:
            f.enabled=bool(value&1) and not bool(value&2)
            if value&2:f.result=None
            if value&4:
                if f.result is None or f.host is not None:raise AssertionError('Invalid result take')
                f.host=f.result;f.result=None
            if value&8:f.host=None
        else:
            if 16<=address<=44 and (f.enabled or f.host or f.active):raise AssertionError('Changed live DDR configuration')
            f.registers[address]=value


class LDSNetBackendTests(unittest.TestCase):
    def setUp(self):
        self.fabric=Fabric();self.backend=LDSNetPerception(self.fabric.nn,self.fabric.display,model_directory=ROOT/'models/lds_net',allocator=self.fabric.allocate,decoder=self.fabric.decode,timeout=.2)
    def tearDown(self):
        self.fabric.hang_clear=False;self.backend.close()
    def wait(self,predicate):
        deadline=time.monotonic()+1
        while not predicate():
            if self.backend.error is not None:raise self.backend.error
            if time.monotonic()>deadline:self.fail('Backend mock timeout')
            time.sleep(.001)
    def test_initialization_is_once_and_normal_frame_only_copies_detection(self):
        self.backend.start()
        self.assertEqual([len(v) for v in self.fabric.buffers],list(SIZES))
        program=(ROOT/'models/lds_net/mac_program.bin').read_bytes()
        self.assertEqual(self.fabric.buffers[3][:len(program)].tobytes(),program)
        self.fabric.inject(0,100);result=self.backend.perception(timeout=1)
        self.assertEqual(result['frame_id'],100);self.assertIsNone(result['mask'])
        self.assertEqual(self.fabric.decoded,[(7000,np.dtype('int8'),.25)])
        invalidations=[v for v in self.fabric.cache if v[0]=='invalidate']
        self.assertEqual(invalidations,[('invalidate',self.fabric.buffers[4].device_address,7040)])
        self.assertEqual(self.fabric.display_registers[0x100],80)
        self.assertEqual(self.fabric.display_registers[0x104],559|(479<<16))
        self.backend.stop();self.backend.start();self.fabric.inject(0,101);self.wait(lambda:self.backend._count==2)
        self.assertEqual(len(self.fabric.buffers),8)
    def test_optional_mask_is_copied_before_publish_and_matches_detection_frame(self):
        self.backend.start();results=[];errors=[]
        def request():
            try:results.append(self.backend.perception(include_mask=True,timeout=1))
            except Exception as error:errors.append(error)
        requester=threading.Thread(target=request);requester.start();self.wait(lambda:self.backend._mask_requested==1)
        self.fabric.inject(0,77);requester.join(1)
        self.assertFalse(requester.is_alive());self.assertFalse(errors)
        result=results[0];self.assertEqual(result['frame_id'],77);self.assertEqual(result['mask'].shape,(480,640))
        np.testing.assert_array_equal(result['mask'][0,80:84],np.array([0,1,2,0],np.uint8))
        self.assertTrue(np.all(result['mask'][:,:80]==255));self.assertTrue(np.all(result['mask'][:,560:]==255))
        # PL may reuse the old mask once the next result commits. The returned
        # Python array must remain an independent copy of its original frame.
        self.fabric.inject(1,78);self.wait(lambda:self.backend._count==2)
        self.fabric.buffers[6][:]=0
        self.assertEqual(int(result['mask'][0,81]),1)
        self.assertEqual([v[2] for v in self.fabric.cache if v[0]=='invalidate'],[7040,57600,7040])
    def test_stopping_joins_and_drains_before_freeing_cma(self):
        self.backend.start();self.fabric.inject(0,90);self.wait(lambda:self.backend._count==1)
        buffers=list(self.fabric.buffers);self.backend.close()
        self.assertFalse(self.fabric.enabled);self.assertIsNone(self.fabric.active);self.assertTrue(all(v.freed for v in buffers))
    def test_timeout_retains_buffers_until_owners_actually_drain(self):
        self.backend.start();self.fabric.inject(0,91);self.wait(lambda:self.backend._count==1)
        self.fabric.hang_clear=True
        with self.assertRaises(TimeoutError):self.backend.stop(timeout=.01)
        self.assertTrue(self.backend._poisoned);self.assertTrue(all(not v.freed for v in self.fabric.buffers))
        self.assertTrue(any(v is self.backend for v in LDSNetPerception._retained))
        self.fabric.hang_clear=False;self.backend.close()
        self.assertTrue(all(v.freed for v in self.fabric.buffers))
        self.assertFalse(any(v is self.backend for v in LDSNetPerception._retained))
    def test_wrong_hardware_is_rejected_before_allocation(self):
        self.fabric.sha^=1
        with self.assertRaisesRegex(RuntimeError,'model hash'):
            LDSNetPerception(self.fabric.nn,self.fabric.display,model_directory=ROOT/'models/lds_net',allocator=self.fabric.allocate,decoder=self.fabric.decode)
        self.assertEqual(self.fabric.buffers,[])


if __name__=='__main__':unittest.main()
