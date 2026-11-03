import importlib.util
from pathlib import Path
import threading
import unittest
import numpy as np
from dual_ov5640.ui_render import UIRenderer,UI_BYTES,BEV_BYTES,TILE_BYTES,BRIGHT,DARK
from dual_ov5640.ui_framebuffer import UIFramebuffer,UI_CAPABILITY

ROOT=Path(__file__).resolve().parents[1]


class Buffer(np.ndarray):
    def __new__(cls,address):
        obj=np.zeros(UI_BYTES,np.uint8).view(cls);obj.device_address=address;obj.freed=False;return obj
    def freebuffer(self):self.freed=True


class Display:
    def __init__(self,ack=True):self.regs={0x300:UI_CAPABILITY,0x31c:0,0x304:0};self.ack=ack;self.events=[]
    def read(self,a):return self.regs.get(a,0)
    def write(self,a,v):
        self.regs[a]=v;self.events.append((a,v))
        if a==0x318:
            self.regs[0x304]=1
            if self.ack:self.regs[0x31c]=self.regs[0x30c];self.regs[0x304]=0


class TestPSUI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.renderer=UIRenderer()
        spec=importlib.util.spec_from_file_location('bev_ref',ROOT/'tools/prepare_lds_net_bev_vectors.py')
        cls.ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.ref)

    def test_exact_pixels(self):
        for targets in ((),*self.ref.TARGETS):
            words=[]
            for t in targets:
                x,y,kind=t[:3];grade=t[3] if len(t)>3 else 0;paint=t[4] if len(t)>4 else 0
                words.append((x&1023)|(y<<10)|(kind<<19)|(grade<<21)|(paint<<23))
            output=np.zeros(UI_BYTES,np.uint8);self.renderer.render(output,words,21)
            np.testing.assert_array_equal(output[:BEV_BYTES].reshape(480,640,3),self.ref.reference(targets))
            for k in range(5):
                tile=output[BEV_BYTES+k*TILE_BYTES:BEV_BYTES+(k+1)*TILE_BYTES].reshape(128,192,3)
                color=BRIGHT[k] if 21&(1<<k) else DARK[k]
                np.testing.assert_array_equal(tile,np.where(self.renderer.hud[k,:,:,None],np.array(color,np.uint8),0))

    def test_flush_before_publication_and_safe_retirement(self):
        display=Display();events=[];buffers=[]
        def allocate(n):
            b=Buffer(0x100000+len(buffers)*0x200000);buffers.append(b);return b
        ui=UIFramebuffer(display,allocate,lambda b,n,op:events.append(op),threading.RLock(),self.renderer)
        ui.start();ui.submit(123,456,[0]*16,0)
        import time
        end=time.monotonic()+2
        while ui.count<1 and time.monotonic()<end:time.sleep(.005)
        self.assertEqual(ui.latest['detection_frame'],123);self.assertEqual(events,['flush'])
        previous=ui.count;ui.set_pedestrian(True)
        end=time.monotonic()+2
        while ui.count<=previous and time.monotonic()<end:time.sleep(.005)
        self.assertGreater(ui.count,previous)
        tile=buffers[ui.front][BEV_BYTES:BEV_BYTES+TILE_BYTES].reshape(128,192,3)
        # Inspect actual DMA memory bytes, not the renderer's RGB canvas.
        np.testing.assert_array_equal(tile,np.where(self.renderer.hud[0,:,:,None],np.array(BRIGHT[0][::-1],np.uint8),0))
        self.assertFalse(any(b.freed for b in buffers));ui.stop()
        self.assertTrue(all(b.freed for b in buffers));self.assertFalse(ui.poisoned)

    def test_timeout_retains_dma_memory(self):
        display=Display();buffers=[]
        def allocate(n):
            b=Buffer(0x100000+len(buffers)*0x200000);buffers.append(b);return b
        ui=UIFramebuffer(display,allocate,lambda *a:None,threading.RLock(),self.renderer,timeout=.02)
        ui.start();display.ack=False;ui.submit(1,1,[0]*16)
        import time
        end=time.monotonic()+1
        while not ui.poisoned and time.monotonic()<end:time.sleep(.005)
        ui.stop();self.assertTrue(ui.poisoned);self.assertFalse(any(b.freed for b in buffers))
        self.assertTrue(ui._retained)

    def test_ui_dma_failure_does_not_block_nn_publication(self):
        from test_lds_net_backend import Fabric,ROOT
        from dual_ov5640.lds_net_backend import LDSNetPerception
        from dual_ov5640.bev_geometry import BEVGeometry
        from dual_ov5640.alerts import HUD_CAPABILITY
        fabric=Fabric();ui_mmio=Display()
        class Combined:
            def read(self,a):
                if a==0x30:return UI_CAPABILITY
                if a==0x38:return HUD_CAPABILITY
                return ui_mmio.read(a) if a>=0x300 else fabric.display.read(a)
            def write(self,a,v):
                if 0x200<=a<0x264:raise AssertionError('PS UI must not write retired PL BEV/HUD registers')
                return ui_mmio.write(a,v) if a>=0x300 else fabric.display.write(a,v)
        class Snapshot:
            def sample_next(self,points,**kwargs):
                return dict(centers=points,patches=[np.full((5,5),3) for _ in points],sgm_sequence=17,request_id=1)
        backend=LDSNetPerception(fabric.nn,Combined(),allocator=fabric.allocate,decoder=fabric.decode,timeout=.2)
        backend.configure_bev(Snapshot(),BEVGeometry.from_directory(ROOT/'hardware/calibration'))
        ui=backend.bev_fusion.ui;ui.renderer=self.renderer;ui.timeout=.025
        import time
        def wait(predicate):
            deadline=time.monotonic()+2
            while not predicate():
                if time.monotonic()>deadline:self.fail('PS UI integration timed out')
                time.sleep(.002)
        try:
            backend.start();wait(lambda:ui.count>=1)
            ui_mmio.ack=False
            fabric.inject(0,101);wait(lambda:backend._count==1)
            wait(lambda:ui.poisoned)
            fabric.inject(1,102);wait(lambda:backend._count==2)
            self.assertIsNone(backend.error)
            self.assertEqual(fabric.publications,[(0,101),(1,102)])
            self.assertIsNone(fabric.host)
        finally:backend.close()
        self.assertTrue(all(b.freed for b in fabric.buffers[:8]))
        self.assertFalse(any(b.freed for b in fabric.buffers[8:]))


if __name__=='__main__':unittest.main()
