import time
import unittest
from unittest.mock import patch
import numpy as np
from dual_ov5640.alerts import AlertPolicy,AlertConfig,road_regions,HUD_CAPABILITY
from dual_ov5640.lds_net_backend import LDSNetPerception
from test_alerts import box,roads
from test_lds_net_backend import Fabric


class HUDHoldTests(unittest.TestCase):
    def test_obstacle_and_both_arrows_activate_on_first_frame(self):
        state=AlertPolicy().update(1,[box()],[9],roads(),now=0)
        self.assertEqual(state['bits'],30)

    def test_all_five_hold_then_require_two_new_misses(self):
        p=AlertPolicy()
        self.assertEqual(p.update(1,[box(category=0)],[9],roads(),now=0)['bits'],31)
        for frame,now in ((2,.01),(3,.2),(4,.999),(5,1.0)):
            self.assertEqual(p.update(frame,[],[],None,now=now)['bits'],31)
        self.assertEqual(p.update(6,[],[],None,now=1.01)['bits'],0)

    def test_positive_frame_breaks_cancellation_streak(self):
        p=AlertPolicy();p.update(1,[box()],[15],roads(),now=0)
        p.update(2,[],[],None,now=1.0)
        self.assertEqual(p.update(3,[box()],[15],roads(),now=1.1)['bits'],26)
        self.assertEqual(p.update(4,[],[],None,now=1.2)['bits'],26)
        self.assertEqual(p.update(5,[],[],None,now=1.3)['bits'],0)

    def test_duplicate_frame_cannot_count_as_second_miss(self):
        p=AlertPolicy();p.update(1,[box()],[15],roads(),now=0)
        p.update(2,[],[],None,now=1.0)
        self.assertTrue(p.update(2,[],[],None,now=1.1)['ignored'])
        self.assertEqual(p.bits,26)
        self.assertEqual(p.update(3,[],[],None,now=1.2)['bits'],0)

    def test_smaller_contiguous_road_is_accepted(self):
        a=np.full((3,80,80),-20,np.int8);a[0]=20
        a[0,50:62,10:15]=-20;a[2,50:62,10:15]=20
        self.assertIsNone(road_regions(a,AlertConfig(min_area=100))[0])
        self.assertEqual(road_regions(a,AlertConfig())[0]['area'],60)
        self.assertEqual(AlertPolicy().update(1,[box()],[9],a,now=0)['bits'],14)

    def test_occupied_side_stays_off_even_with_obstacle(self):
        state=AlertPolicy().update(1,[box(),box(140)],[9,5],roads(),now=0)
        self.assertTrue(state['blocked'][0]);self.assertEqual(state['bits'],22)

    def test_unknown_depth_blocks_new_arrows(self):
        state=AlertPolicy().update(1,[box(),box(220)],[9,None],roads(),now=0)
        self.assertTrue(state['central_unknown']);self.assertEqual(state['bits'],6)

    def test_direct_nn_pedestrian_path_uses_same_hold(self):
        f=Fabric();f.display_registers[0x38]=HUD_CAPABILITY
        detections=[box(category=0)]
        backend=LDSNetPerception(f.nn,f.display,allocator=f.allocate,
                                  decoder=lambda *_:list(detections),timeout=.2)
        try:
            backend.start()
            # Run the production result method synchronously, avoiding sleeps.
            backend._stop.set();backend._thread.join(1);backend._thread=None;backend._stop.clear()
            class Clock:
                value=0
                @classmethod
                def monotonic(cls):return cls.value
            with patch('dual_ov5640.lds_net_backend.time',Clock):
                for frame,now,person,expected in ((1,0,True,1),(2,.1,False,1),(3,.99,False,1),(4,1.0,False,1),(5,1.1,False,0)):
                    Clock.value=now;detections[:]=[box(category=0)] if person else []
                    f.inject((frame-1)%2,frame);backend._process_result()
                    self.assertEqual(f.display_registers[0x260],expected)
        finally:backend.close()


if __name__=='__main__':unittest.main()
