import unittest
import time
import numpy as np
from dual_ov5640.bev_color import (COLOR_RGB,PAINT_INDICES,estimate_color,attach_colors,
    paint_palette,RGB_CAPABILITY,COLOR_CAPABILITY)
from dual_ov5640.lds_net_backend import LDSNetPerception
from test_lds_net_backend import Fabric


BOX=dict(x1=200.,y1=120.,x2=400.,y2=420.,score=.9,category=2)


class ColorTests(unittest.TestCase):
    def test_all_nine_colors_from_signed_rgb_input(self):
        for color in range(1,10):
            rgb=np.empty((320,320,3),np.int8);rgb[:]=np.array(COLOR_RGB[color],np.int16)-128
            result=estimate_color(rgb,BOX)
            self.assertEqual(result['color_class'],color,(color,result))
            self.assertEqual(estimate_color(rgb.view(np.uint8),BOX),result)

    def test_small_or_outside_boxes_are_unknown_not_fixed_red(self):
        rgb=np.full((320,320,3),0,np.int8)
        for box in (dict(BOX,x2=205),dict(BOX,y2=130),dict(BOX,x1=-100,x2=-50)):
            self.assertEqual(estimate_color(rgb,box)['color_class'],0)

    def test_body_palette_preserves_glass_wheels_and_transparency(self):
        from pathlib import Path
        import json
        atlas=json.loads((Path(__file__).resolve().parents[1]/'hardware/assets/bev/rom.json').read_text())
        original=np.array(atlas['palettes'][1],np.uint8)
        for color in range(10):
            palette=paint_palette(original,color)
            np.testing.assert_array_equal(palette[4],COLOR_RGB[color])
            for index in set(range(16))-set(PAINT_INDICES):np.testing.assert_array_equal(palette[index],original[index])


    def test_color_is_attached_to_detection_frame_and_survives_selection(self):
        from dual_ov5640.bev_geometry import select_targets,BEVGeometry
        from pathlib import Path
        rgb=np.empty((320,320,3),np.int8);rgb[:]=np.array(COLOR_RGB[5],np.int16)-128
        boxes=attach_colors([BOX,dict(BOX,category=0)],rgb,42)
        self.assertNotIn('vehicle_color',BOX);self.assertNotIn('vehicle_color',boxes[1])
        selected=select_targets(boxes)
        self.assertEqual(selected[0]['vehicle_color']['source_frame'],42)
        geometry=BEVGeometry.from_directory(Path(__file__).resolve().parents[1]/'hardware/calibration')
        words,targets=geometry.project(selected,dict(centers=[(300,270)]*2,patches=[np.full((5,5),3)]*2))
        self.assertEqual((words[0]>>23)&15,5)
        self.assertEqual(targets[0]['vehicle_color']['color_class'],5)


class RGBMMIO:
    def __init__(self,fabric):self.fabric=fabric;self.base=fabric.nn;self.enabled_color=0;self.bad_frame=False;self.lose=False;self.reads=0
    def read(self,address):
        f=self.fabric
        if address==0xc4:return RGB_CAPABILITY
        if address==0xc0:return self.enabled_color
        if address in (0xc8,0xcc):
            if f.host is None or not self.enabled_color:return 0
            if address==0xc8:
                self.reads+=1
                if self.lose and self.reads>1:return 0
                return f.buffers[1-f.host[0]].device_address
            return f.host[1]+int(self.bad_frame)
        return self.base.read(address)
    def write(self,address,value):
        if address==0xc0:
            if self.fabric.enabled:raise AssertionError('Live RGB mode changed')
            self.enabled_color=value;return
        self.base.write(address,value)


class RGBBackendTests(unittest.TestCase):
    def setUp(self):
        self.f=Fabric();self.f.display_registers.update({0x30:0x42455632,0x34:COLOR_CAPABILITY})
        self.mmio=RGBMMIO(self.f)
        self.backend=LDSNetPerception(self.mmio,self.f.display,allocator=self.f.allocate,decoder=lambda *a:[dict(BOX)],timeout=.2)
        class Fusion:
            def start(self):pass
            def stop(self,timeout=None):pass
            def submit(self,frame,boxes):self.received=(frame,boxes)
        self.backend.bev_fusion=Fusion();self.backend.start()
    def tearDown(self):self.backend.close()
    def inject(self):
        # Input 1 belongs to output 0; input/output slot identity must not be assumed.
        rgb=np.array(COLOR_RGB[4],np.int16)-128
        self.f.buffers[1][:]=np.tile(rgb.astype(np.int8).view(np.uint8),102400)
        self.f.inject(0,123)
    def wait(self):
        deadline=time.monotonic()+1
        while not self.backend._count and self.backend.error is None and time.monotonic()<deadline:time.sleep(.001)
    def test_reads_owned_input_slot_before_publish(self):
        self.inject();self.wait();self.assertIsNone(self.backend.error)
        box=self.backend.detections()[0]
        self.assertEqual(box['vehicle_color']['color_class'],4)
        self.assertEqual(box['vehicle_color']['source_frame'],123)
        self.assertIn(('invalidate',self.f.buffers[1].device_address,307200),self.f.cache)
        self.assertEqual(self.f.publications,[(0,123)])
        self.assertEqual(self.backend.bev_fusion.received[1][0]['vehicle_color']['color_class'],4)
    def test_wrong_frame_is_rejected_without_publication(self):
        self.mmio.bad_frame=True;self.inject();self.wait()
        self.assertIn('not owned',str(self.backend.error));self.assertFalse(self.f.publications)
    def test_lost_ownership_after_sampling_is_rejected(self):
        self.mmio.lose=True;self.inject();self.wait()
        self.assertIn('ownership changed',str(self.backend.error));self.assertFalse(self.f.publications)
