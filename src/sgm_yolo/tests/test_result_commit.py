import unittest
from unittest.mock import Mock
from dual_ov5640.result_commit import ResultCommitter
from dual_ov5640.overlay import SGMYOLOOverlay


class Memory:
    def __init__(self):
        self.values={0xE0:0x80000001,0xE4:7,0xE8:6}
        self.writes=[]
    def read(self,offset): return self.values.get(offset,0)
    def write(self,offset,value):
        self.values[offset]=value
        self.writes.append((offset,value))


class ResultCommitTests(unittest.TestCase):
    def test_request_is_not_acknowledgement(self):
        mmio=Memory();reader=Mock();reader.current_frame.return_value=0
        committer=ResultCommitter(mmio,reader)
        self.assertFalse(committer.poll())
        reader.park_frame.assert_called_once_with(1)
        self.assertEqual(mmio.writes,[])
        reader.current_frame.return_value=1
        self.assertTrue(committer.poll())
        self.assertEqual(mmio.writes,[(0xE8,7)])
        self.assertFalse(committer.poll())

    def test_uncommitted_frames_are_not_selected(self):
        mmio=Memory();mmio.values[0xE0]=1;reader=Mock()
        self.assertFalse(ResultCommitter(mmio,reader).poll())
        reader.park_frame.assert_not_called()
        self.assertEqual(mmio.writes,[])

    def test_invalid_index_cannot_release_writer(self):
        mmio=Memory();mmio.values[0xE0]=0x80000003
        with self.assertRaisesRegex(RuntimeError,'Invalid committed'):
            ResultCommitter(mmio,Mock()).poll()
        self.assertEqual(mmio.writes,[])

    def test_recovery_starts_reader_on_retained_committed_frame(self):
        driver=object.__new__(SGMYOLOOverlay)
        mmio=Memory();mmio.values[0]=0x53474D31
        driver._mmios={'axi_vdma_disp':mmio}
        driver.result_committer=Mock();driver.disparity_read=Mock()
        driver._configure_result_reader((0x1000,0x2000,0x3000))
        driver.disparity_read.configure.assert_called_once_with(
            (0x1000,0x2000,0x3000),640,480,640,parked=True,frame_index=1)
        driver.result_committer.start.assert_called_once_with()
        self.assertEqual(mmio.writes,[])
