import unittest
import numpy as np
from dual_ov5640.palette import colorize_disparity,distance_to_rgb,FOCAL_BASELINE_M_PX


class PaletteTest(unittest.TestCase):
    def test_fixed_depth_direction_and_rgb_order(self):
        result = colorize_disparity(np.array([0,1,29,63]))
        np.testing.assert_array_equal(result, [[0,0,128], [0,1,255],
                                               [255,131,0], [255,0,0]])

    def test_color_is_independent_of_other_pixels(self):
        np.testing.assert_array_equal(colorize_disparity([21])[0],
                                      colorize_disparity([0,21,63])[1])

    def test_exact_distance_anchors_and_smooth_midpoints(self):
        np.testing.assert_array_equal(distance_to_rgb([1,2.5,5,10]),
            [[255,128,0],[255,255,0],[0,255,0],[0,255,255]])
        np.testing.assert_array_equal(distance_to_rgb([1.75,3.75,7.5]),
            [[255,192,0],[128,255,0],[0,255,128]])
        for anchor in (1,2.5,5,10):
            nearby=distance_to_rgb([anchor-.001,anchor+.001]).astype(int)
            self.assertLessEqual(np.abs(nearby[0]-nearby[1]).max(),1)
        np.testing.assert_array_equal(colorize_disparity(np.arange(1,64)),
                                      distance_to_rgb(FOCAL_BASELINE_M_PX/np.arange(1,64)))

    def test_python_and_all_hardware_lut_entries_match(self):
        from pathlib import Path
        import re
        source=(Path(__file__).resolve().parents[1]/'hardware/rtl/triple_rgb_compositor.v').read_text()
        entries=re.findall(r"6'd(\d+): jet_rgb=24'h([0-9a-f]{6})",source)
        self.assertEqual(len(entries),64)
        for index,hexrgb in entries:
            np.testing.assert_array_equal(colorize_disparity([int(index)])[0],list(bytes.fromhex(hexrgb)))
