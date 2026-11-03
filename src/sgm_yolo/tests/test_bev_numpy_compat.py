"""Exercise deployed HUD + BEV workers with the older required-mode pad API."""
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from dual_ov5640.alerts import road_regions,AlertConfig,HUD_CAPABILITY
from dual_ov5640.bev_geometry import BEVGeometry
from dual_ov5640.bev_fusion import BEVFusion
from dual_ov5640.sgm_snapshot import SGMSnapshot
from dual_ov5640.ui_framebuffer import UIFramebuffer
from dual_ov5640.ui_render import UIRenderer,UI_BYTES,BEV_BYTES
from test_alerts import roads
from test_bev_depth import SnapshotMMIO,detection
from test_ps_ui import Buffer,Display

ROOT=Path(__file__).resolve().parents[1]
ORIGINAL_PAD=np.pad


def required_mode_pad(array,pad_width,mode,**kwargs):
    # Reproduce the old NumPy signature without installing/changing NumPy.
    return ORIGINAL_PAD(array,pad_width,mode=mode,**kwargs)


class HUDDisplay(Display):
    def read(self,address):
        return HUD_CAPABILITY if address==0x38 else super().read(address)


class BEVNumPyCompatibilityTests(unittest.TestCase):
    def test_road_components_keep_identical_constant_zero_padding(self):
        logits=roads()
        with patch('dual_ov5640.alerts.np.pad',required_mode_pad):
            left,right=road_regions(logits,AlertConfig())
        self.assertEqual((left['xmin'],left['xmax']),(92,188))
        self.assertEqual((right['xmin'],right['xmax']),(458,548))
        # Compare against explicitly padded data to protect the edge semantics.
        a=(logits.argmax(0)>0).astype(np.uint8)
        expected=np.zeros((82,82),np.uint8);expected[1:-1,1:-1]=a
        np.testing.assert_array_equal(required_mode_pad(a,1,'constant',constant_values=0),expected)

    def test_enabled_hud_semantics_allows_person_to_reach_ui_ack(self):
        renderer=UIRenderer();display=HUDDisplay();buffers=[]
        geometry=BEVGeometry.from_directory(ROOT/'hardware/calibration')
        mmio=SnapshotMMIO();encoded=(4<<2)|(4>>4)
        frames=[np.full((480,640),encoded,np.uint8) for _ in range(3)]
        def allocate(n):
            value=Buffer(0x100000+len(buffers)*0x200000);buffers.append(value);return value
        ui=UIFramebuffer(display,allocate,lambda *a:None,threading.RLock(),renderer)
        worker=BEVFusion(SGMSnapshot(mmio,frames),geometry,display,ui.io,ui)
        with patch('dual_ov5640.alerts.np.pad',required_mode_pad):
            try:
                self.assertTrue(worker.alert_enabled)
                worker.start();worker.submit(91,[detection(category=0)],roads())
                deadline=time.monotonic()+3
                while not (ui.latest and ui.latest['detection_frame']==91):
                    if worker.error or ui.error:self.fail(str((worker.error,ui.error)))
                    if time.monotonic()>deadline:self.fail('HUD + person UI acknowledgement timed out')
                    time.sleep(.002)
                self.assertEqual(worker.count,1);self.assertIsNone(worker.error)
                target=worker.latest['targets'][0]
                self.assertTrue(target['valid']);self.assertAlmostEqual(target['distance_m'],geometry.fb/4)
                self.assertEqual(worker.latest['alerts']['bits'],31)
                self.assertEqual(ui.latest['sgm_sequence'],8)
                self.assertEqual((int(ui.scene[2][0])>>19)&3,2)
                # An acknowledged frame contains the actual person sprite,
                # with the completed atlas packed to the BGR DMA byte order.
                rgb=np.asarray(buffers[ui.front])[:BEV_BYTES].reshape(480,640,3)[:,:,::-1]
                empty=np.empty(UI_BYTES,np.uint8);renderer.render(empty,[],7)
                self.assertGreater(np.count_nonzero(rgb!=empty[:BEV_BYTES].reshape(480,640,3)),20)
                self.assertEqual(mmio.read(0xfc),0)
            finally:worker.stop()
        self.assertTrue(all(b.freed for b in buffers))


if __name__=='__main__':unittest.main()
