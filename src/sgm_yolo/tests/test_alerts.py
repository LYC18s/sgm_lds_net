import unittest
import numpy as np
from dual_ov5640.alerts import AlertPolicy,road_regions,AlertConfig

def box(x=320,category=2):return dict(x1=x-30,x2=x+30,y1=200,y2=300,category=category,score=.8)
def roads():
 a=np.full((3,80,80),-20,np.int8);a[0]=20
 for sl in (slice(2,18),slice(63,78)):
  a[0,36:76,sl]=-20;a[2,36:76,sl]=20
 return a
class AlertTests(unittest.TestCase):
 def test_thresholds_and_arrows(self):
  p=AlertPolicy();self.assertEqual(p.update(1,[box()],[19],roads(),now=0)['bits'],26)
  self.assertEqual(p.update(2,[box()],[19],roads(),now=.1)['bits'],26)
  self.assertEqual(p.update(3,[box()],[9],roads(),now=.2)['bits'],30)
 def test_center_40_percent_and_light(self):
  self.assertEqual(AlertPolicy().update(1,[box(200)],[19],None)['bits'],2)
  self.assertEqual(AlertPolicy().update(1,[box(190)],[9],None)['bits'],0)
  self.assertEqual(AlertPolicy().update(1,[box(320,6)],[],roads())['bits'],0)
 def test_center_only_not_box_intersection(self):
  p=AlertPolicy();a=roads();b=[box(),box(220)]
  p.update(1,b,[15,30],a,now=0);self.assertEqual(p.update(2,b,[15,30],a,now=.1)['bits'],26)
  b=[box(),box(140)];self.assertEqual(p.update(3,b,[15,30],a,now=1.0)['bits'],26)
  self.assertEqual(p.update(4,b,[15,30],a,now=1.1)['bits'],18)
 def test_person_without_depth_and_all_classes(self):
  self.assertEqual(AlertPolicy().update(1,[box(320,0)],[None],None)['bits'],1)
  for cat in (1,3,7,8,9):self.assertEqual(AlertPolicy().update(1,[box(320,cat)],[15],None)['bits'],2)
 def test_noise_and_near_connection(self):
  a=roads();a[2]=np.where(np.indices((80,80)).sum(0)%2,20,-20);a[0]=0
  self.assertEqual(road_regions(a,AlertConfig()),[None,None])
  a=roads();a[:,55:]=0;self.assertEqual(road_regions(a,AlertConfig()),[None,None])
 def test_hysteresis_unknown_and_recovery(self):
  p=AlertPolicy();p.update(1,[box()],[19],roads(),now=0)
  self.assertEqual(p.update(2,[box()],[21],roads(),now=.1)['bits']&2,2)
  self.assertEqual(p.update(3,[box()],[None],roads(),now=.2)['bits']&24,24)
  self.assertEqual(p.update(4,[],[],None,now=.3)['bits']&2,2)
  self.assertEqual(p.update(5,[],[],None,now=1.0)['bits'],26)
  self.assertEqual(p.update(6,[],[],None,now=1.1)['bits'],0)
 def test_same_frame_no_confirmation_and_stale(self):
  p=AlertPolicy();p.update(2,[box()],[15],roads(),now=0)
  self.assertTrue(p.update(2,[box()],[15],roads(),now=.1)['ignored'])
  self.assertTrue(p.update(1,[box()],[15],roads(),now=.2)['ignored'])
  self.assertEqual(p.update(3,[box()],[15],roads(),now=4)['bits'],26)
 def test_complete_regions_cross_warning_boundaries(self):
  a=roads()
  for region in (slice(6,25),slice(55,74)):
   a[0,36:76,region]=-20;a[2,36:76,region]=20
  left,right=road_regions(a,AlertConfig())
  self.assertEqual((left['xmin'],left['xmax']),(92,230))
  self.assertEqual((right['xmin'],right['xmax']),(410,548))
  p=AlertPolicy();b=[box(),box(193.3)]
  p.update(1,b,[15,16],a)
  state=p.update(2,b,[15,16],a)
  self.assertTrue(state['blocked'][0]);self.assertFalse(state['bits']&8)
  p=AlertPolicy();b=[box(),box(447)]
  p.update(1,b,[15,16],a)
  state=p.update(2,b,[15,16],a)
  self.assertTrue(state['blocked'][1]);self.assertFalse(state['bits']&16)
 def test_direct_road_does_not_bridge_alternative_regions(self):
  a=roads();a[0,36:76,18:63]=-20;a[1,36:76,18:63]=20
  left,right=road_regions(a,AlertConfig())
  self.assertEqual((left['xmin'],left['xmax']),(92,188))
  self.assertEqual((right['xmin'],right['xmax']),(458,548))
  # The warning region must not determine how road regions are cut or assigned.
  self.assertEqual([left,right],road_regions(a,AlertConfig(center_min=240,center_max=400)))
if __name__=='__main__':unittest.main()
