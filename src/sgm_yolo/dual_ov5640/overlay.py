"""SGM-YOLO: live stereo, left RGB capture, disparity and HDMI display."""

import time
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .camera import OV5640
from .clock import DynamicClock
from .vdma import VDMAChannel, VDMA_STATUS_ERROR_MASK
from .vtc import MODE_1080P60, VideoTimingController
from .result_commit import ResultCommitter


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
OUTPUT_WIDTH = 1920
OUTPUT_HEIGHT = 1080
BYTES_PER_PIXEL = 3
STRIDE = CAMERA_WIDTH * BYTES_PER_PIXEL
DRIVER_BUILD = "2026-11-02-lds_net-refocus15"


class SGMYOLOOverlay:
    """Capture left RGB888 and compute live 64-disparity stereo SGM in PL.

    HDMI displays centered left RGB, disparity and BEV panels with warning icons. The
    camera.frame() interface remains available for Python YOLO processing.
    """

    IP_NAMES = (
        "axi_vdma_0", "axi_vdma_1", "axi_vdma_2", "axi_vdma_3",
        "axi_dynclk_0", "v_tc_0", "cmos_rst",
        "axi_vdma_disp", "axi_vdma_disp_rd",
    )

    def __init__(
        self,
        bitfile=None,
        *,
        i2c_buses=(0, 1),
        download: bool = True,
        enable_lds_net: bool = True,
    ):
        if bitfile is None:
            bitfile = Path(__file__).resolve().parents[1] / "overlay" / "sgm_yolo.bit"
        if not Path(bitfile).exists():
            raise FileNotFoundError("SGM bitstream is missing. Run build/build.py --stage route and --stage export from the repository root: " + str(bitfile))
        try:
            from pynq import Clocks, MMIO, Overlay
        except ImportError as exc:
            raise RuntimeError("This driver must run on a PYNQ Linux image") from exc

        self.bitfile = Path(bitfile)
        manifest_file = self.bitfile.with_suffix(".build.json")
        self.build_manifest = json.loads(manifest_file.read_text(encoding="utf-8")) if manifest_file.exists() else {}
        if not self.bitfile.with_suffix(".hwh").exists():
            raise FileNotFoundError(f"Missing HWH beside bitstream: {self.bitfile.with_suffix('.hwh')}")
        self._hwh_root = ET.parse(self.bitfile.with_suffix(".hwh")).getroot()
        modules = {m.get("INSTANCE"): m for m in self._hwh_root.findall(".//MODULE")}
        self.lds_net_available = 'lds_net_perception_0' in modules and 'lds_net_display_0' in modules
        self.enable_lds_net = enable_lds_net
        if enable_lds_net and self.lds_net_available:
            from .hardware_contract import validate_lds_net_reset
            validate_lds_net_reset(self._hwh_root)
        self.detector = None
        def parameters(name):
            module = modules.get(name)
            return {} if module is None else {
                p.get("NAME"): p.get("VALUE") for p in module.findall("./PARAMETERS/PARAMETER")
            }
        # HWH selects the capture ABI, allowing archived complete packages to
        # remain usable while the compact build allocates left RGB only.
        self.right_rgb_available = "axi_vdma_0" in modules and "axi_vdma_1" in modules
        if ("axi_vdma_0" in modules) != ("axi_vdma_1" in modules):
            raise ValueError("Incomplete right RGB DMA pair in HWH")
        self.camera_input_filter = "RGB per-channel 3x3 median" if "cam0_median" in modules and "cam1_median" in modules else "none"
        if not self.right_rgb_available and any(name in modules for name in ("cam0_median", "cam1_median")):
            raise ValueError("Compact hardware must remove both camera RGB median filters")
        self.IP_NAMES = tuple(name for name in type(self).IP_NAMES
                              if self.right_rgb_available or name not in ("axi_vdma_0", "axi_vdma_1"))
        for name in ("cam0_axis_bcast", "cam1_axis_bcast"):
            params = parameters(name)
            if params.get("DROP_STALLED_SGM") != "1":
                raise ValueError("Upload the complete 2026-10-08 live-SGM board package (matching bit/HWH and driver). This HWH describes the older blocking camera branch.")
        layout = parameters("triple_rgb_compositor_0")
        from .hardware_contract import validate_hdmi_layout
        self.hdmi_layout = validate_hdmi_layout(
            self._hwh_root, panel_y=self.build_manifest.get('hud', {}).get('panel_y'))
        sgm_params = parameters("stereo_sgm_0")
        self.sgm_core_lanes = int(sgm_params.get("CORE_LANES", "64"))
        self.sgm_pixel_ii = int(sgm_params.get("CORE_PIXEL_II", "1"))
        if sgm_params.get("SWAP_INPUTS") != "1" or layout.get("SWAP_RGB_INPUTS") != "1":
            raise ValueError("Upload the complete stereo-order-fixed bit/HWH and driver. Logical left must be physical camera 1 for this fixed rig.")
        if layout.get("COLOR_DISPARITY") != "1":
            raise ValueError("Upload the matching color-disparity bit/HWH and driver; this hardware has the older gray HDMI panel.")
        if sgm_params.get("RECTIFY_INPUTS") != "1":
            raise ValueError("Upload the complete live-calibrated-SGM bit/HWH and driver; this hardware does not rectify the camera inputs.")
        self.streaming_result = parameters("axi_vdma_disp").get("TRANSACTIONAL_RESULT") == "1"
        if self.streaming_result and sgm_params.get("STREAM_OUTPUT") != "1":
            raise ValueError("Streaming result DMA requires matching STREAM_OUTPUT SGM hardware")
        self.physical_camera_order = (1, 0)
        self.calibration_directory = Path(__file__).resolve().parents[1] / "hardware" / "calibration"
        self._calibration_maps = None

        from .lds_net_backend import LDSNetPerception
        LDSNetPerception.stop_existing()
        ResultCommitter.stop_existing()
        self.overlay = Overlay(str(self.bitfile), download=download)
        self._MMIO = MMIO
        self._Clocks = Clocks
        self._i2c_buses = tuple(i2c_buses)
        if len(self._i2c_buses) != 2:
            raise ValueError("Exactly two I2C buses are required")

        self._mmios = {name: self._make_mmio(name) for name in self.IP_NAMES}
        self.clock = DynamicClock(self._mmios["axi_dynclk_0"])
        self.vtc = VideoTimingController(self._mmios["v_tc_0"])
        self.read_channels = (
            VDMAChannel(self._mmios["axi_vdma_2"], "read"),
            *((VDMAChannel(self._mmios["axi_vdma_0"], "read"),) if self.right_rgb_available else ()),
        )
        self.write_channels = (
            VDMAChannel(self._mmios["axi_vdma_3"], "write", flush_on_fsync=
                int(parameters('axi_vdma_3').get('C_FLUSH_ON_FSYNC', '0')) in (1, 3)),
            *((VDMAChannel(self._mmios["axi_vdma_1"], "write", flush_on_fsync=
                int(parameters('axi_vdma_1').get('C_FLUSH_ON_FSYNC', '0')) in (1, 3)),) if self.right_rgb_available else ()),
        )
        self.disparity_write = VDMAChannel(self._mmios["axi_vdma_disp"], "write")
        self.disparity_read = VDMAChannel(self._mmios["axi_vdma_disp_rd"], "read")
        self.result_committer = ResultCommitter(self._mmios["axi_vdma_disp"], self.disparity_read, self.stop) if self.streaming_result else None
        self.disparity_frames = None
        self.cameras = tuple(OV5640(self._i2c_buses[i]) for i in self.physical_camera_order)
        self.frames = None
        self.actual_pixel_clock_mhz = None
        self.running = False

    def _ip_description(self, name: str):
        if name in self.overlay.ip_dict:
            return self.overlay.ip_dict[name]
        matches = [desc for key, desc in self.overlay.ip_dict.items() if key.rsplit("/", 1)[-1] == name]
        lds_net = name in ('lds_net_perception_0', 'lds_net_display_0')
        if not matches and ((name == "axi_vdma_disp" and self.streaming_result) or lds_net):
            # Older PYNQ parsers can omit module_ref peripherals from ip_dict.
            # Read the CPU register window, not this DMA's DDR master window.
            ranges = self._hwh_root.findall(
                ".//MODULE[@MODTYPE='processing_system7']/MEMORYMAP/MEMRANGE"
            )
            matches = [r for r in ranges if r.get("INSTANCE") == name
                       and r.get("MEMTYPE") == "REGISTER"
                       and r.get("MASTERBUSINTERFACE") == "M_AXI_GP0"
                       and r.get("SLAVEBUSINTERFACE") == ("S_AXI" if lds_net else "S_AXI_LITE")]
            if len(matches) != 1:
                raise KeyError(f"Expected one CPU register range for {name!r}, found {len(matches)}")
            base = int(matches[0].get("BASEVALUE"), 0)
            high = int(matches[0].get("HIGHVALUE"), 0)
            required = 0x17F if name == 'lds_net_display_0' else 0xBF if lds_net else 0xEB
            if base <= 0 or base % 4 or high < base + required or high >= (1 << 32):
                raise ValueError(f"Invalid HWH register range for {name!r}")
            return {"phys_addr": base, "addr_range": high - base + 1}
        if len(matches) != 1:
            raise KeyError(f"Expected one {name!r} in HWH, found {len(matches)}")
        return matches[0]

    def _make_mmio(self, name: str):
        desc = self._ip_description(name)
        return self._MMIO(desc["phys_addr"], desc["addr_range"])

    @staticmethod
    def _allocate_frame():
        shape = (CAMERA_HEIGHT, CAMERA_WIDTH, BYTES_PER_PIXEL)
        try:
            from pynq import allocate
            return allocate(shape=shape, dtype=np.uint8)
        except ImportError:
            # Compatibility with older vendor images based on PYNQ 2.4.
            from pynq import Xlnk
            return Xlnk().cma_array(shape=shape, dtype=np.uint8, cacheable=1)

    @staticmethod
    def _device_address(buffer) -> int:
        if hasattr(buffer, "device_address"):
            return int(buffer.device_address)
        return int(buffer.physical_address)

    @staticmethod
    def _flush(buffer) -> None:
        if hasattr(buffer, "flush"):
            buffer.flush()

    @staticmethod
    def _invalidate(buffer) -> None:
        if hasattr(buffer, "invalidate"):
            buffer.invalidate()

    @staticmethod
    def _allocate_disparity_frame():
        shape = (CAMERA_HEIGHT, CAMERA_WIDTH)
        try:
            from pynq import allocate
            return allocate(shape=shape, dtype=np.uint8)
        except ImportError:
            from pynq import Xlnk
            return Xlnk().cma_array(shape=shape, dtype=np.uint8, cacheable=1)

    def reset_cameras(self) -> None:
        gpio = self._mmios["cmos_rst"]
        gpio.write(0x04, 0x0)  # channel 1 direction: both outputs
        gpio.write(0x00, 0x3)
        time.sleep(0.5)
        gpio.write(0x00, 0x0)
        time.sleep(0.5)
        gpio.write(0x00, 0x3)
        time.sleep(0.5)

    def start(self, *, initialize_sensors=True, verify_sensor_ids=True):
        if self.running:
            return
        self._preflight_lds_net_snapshot()
        if self.disparity_frames is None:
            self.disparity_frames = tuple(self._allocate_disparity_frame() for _ in range(3))
            for frame in self.disparity_frames:
                frame[:] = 0
                self._flush(frame)
        addresses = tuple(self._device_address(frame) for frame in self.disparity_frames)
        # Arm the result sink first; live RGB capture remains independent.
        try:
            self.vtc.disable()
            self._configure_sgm_clock()
            self.disparity_write.configure(addresses, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_WIDTH)
            if getattr(self, "streaming_result", False):
                self._configure_result_reader(addresses)
            else:
                self.disparity_read.configure(addresses, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_WIDTH)
            self._start_cameras(
                initialize_sensors=initialize_sensors,
                verify_sensor_ids=verify_sensor_ids,
            )
            # Buffers have been allocated/initialized and both readers armed.
            # The raster must run before waiting for SGM: recovery preserves a
            # published frame but clears the completion IRQ. Its parked reader
            # can become active (and acknowledge that frame) only as HDMI drains
            # the stream. Waiting with VTC disabled leaves the writer in WAIT_ACK.
            self.vtc.enable()
            self._check_startup(initialize_sensors, verify_sensor_ids)
            if getattr(self, 'enable_lds_net', False):
                self.start_lds_net()
        except Exception:
            self.stop()
            raise

    def _preflight_lds_net_snapshot(self):
        # Check the loaded hardware before allocating CMA, starting sensors or
        # enabling HDMI. An HWH declaration alone cannot prove a cached DCP
        # actually implements the depth-frame lease registers.
        if not (getattr(self,'enable_lds_net',False) and getattr(self,'lds_net_available',False)):
            return
        from .ui_framebuffer import UI_CAPABILITY
        from .bev_fusion import BEV2_CAPABILITY
        display=self._make_mmio('lds_net_display_0')
        if display.read(0x300)==UI_CAPABILITY or display.read(0x30)==BEV2_CAPABILITY:
            from .sgm_snapshot import SGMSnapshot
            SGMSnapshot.require_capability(self.disparity_write.mmio)

    def _configure_result_reader(self, addresses):
        writer = self._mmios["axi_vdma_disp"]
        if writer.read(0) != 0x53474D31:
            raise RuntimeError("Transactional result DMA ABI mismatch")
        self.result_committer.stop()
        published = writer.read(0xE0)
        index = published & 3 if published & 0x80000000 else 0
        if index >= 3:
            raise RuntimeError("Invalid committed disparity buffer")
        # A retained committed frame may still await acknowledgement after a
        # stop/recovery. Start reading that frame immediately; waiting for HDMI
        # before selecting it would deadlock the first-frame startup check.
        self.disparity_read.configure(addresses, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_WIDTH,
                                      parked=True, frame_index=index)
        self.result_committer.start()

    def _start_cameras(self, *, initialize_sensors: bool = True, verify_sensor_ids: bool = True) -> None:
        if self.running:
            return
        if self.frames is None:
            self.frames = tuple(
                tuple(self._allocate_frame() for _ in range(3))
                for _ in self.write_channels
            )
            for camera_frames in self.frames:
                for frame in camera_frames:
                    frame[:] = 0
                    self._flush(frame)

        self.reset_cameras()
        if initialize_sensors:
            for camera in self.cameras:
                camera.initialize_vga_rgb565(verify_id=verify_sensor_ids, start_streaming=False)

        self.actual_pixel_clock_mhz = self.clock.configure(MODE_1080P60.pixel_clock_mhz)
        self.vtc.configure(MODE_1080P60, enable=False)

        hsize = CAMERA_WIDTH * BYTES_PER_PIXEL
        addresses = tuple(
            tuple(self._device_address(frame) for frame in camera_frames)
            for camera_frames in self.frames
        )
        for channel, frame_addresses in zip(self.write_channels, addresses):
            channel.configure(frame_addresses, hsize, CAMERA_HEIGHT, STRIDE)
        for channel, frame_addresses in zip(self.read_channels, addresses):
            channel.configure(frame_addresses, hsize, CAMERA_HEIGHT, STRIDE)
        if initialize_sensors:
            # Configure both sensors in standby, arm the capture paths, then
            # enable the two streams consecutively. Avoid a register-table and
            # settling-delay skew between the two first frame starts.
            for camera in self.cameras:
                camera.write(0x3008, 0x02)
            time.sleep(0.020)
            for index in range(len(self.cameras)):
                self._confirm_streaming(index)
        self.running = True

    def _confirm_streaming(self, index):
        sensor = self.cameras[index]
        for attempt in range(3):
            actual = sensor.read(0x3008)
            if actual == 0x02:
                return
            if attempt < 2:
                sensor.write(0x3008, 0x02)
                time.sleep(0.020)
        raise RuntimeError("Logical camera %d failed stream-on: 0x3008=0x%02x" % (index, actual))

    def _configure_sgm_clock(self):
        target_mhz=100.0 if getattr(self,'sgm_core_lanes',32)==16 else 50.0
        self._Clocks.fclk3_mhz = target_mhz
        self.actual_sgm_clock_mhz = float(self._Clocks.fclk3_mhz)
        if abs(self.actual_sgm_clock_mhz - target_mhz) > 0.5:
            raise RuntimeError("SGM FCLK3 configuration mismatch: %.3f MHz" % self.actual_sgm_clock_mhz)

    def _check_startup(self, initialize_sensors, verify_sensor_ids):
        """Require complete captures; restart the pair together at most once."""
        self.startup_recovery = []
        self.startup_pair_restart = False
        self.startup_capture_recovery = []
        self.startup_failure = None
        try:
            self.wait_for_frames(timeout=15.0, recover_capture_errors=True)
            return
        except TimeoutError:
            missing = [i for i, channel in enumerate(self.write_channels)
                       if not (channel.status() & 0x1000) or channel.errors()]
        if initialize_sensors:
            if not getattr(self, "right_rgb_available", True):
                # With no right RGB DMA, a missing depth frame cannot identify
                # which sensor needs reinitializing. Recover both together.
                missing = list(range(len(self.cameras)))
            # Do not leave the healthy side streaming while the other sensor's
            # register table is replayed: the SGM input FIFOs cannot absorb an
            # arbitrary difference between the two first-frame start times.
            for sensor in self.cameras:
                sensor.write(0x3008, 0x42)
            time.sleep(0.020)
            for index in missing:
                sensor = self.cameras[index]
                sensor.initialize_vga_rgb565(verify_id=verify_sensor_ids, start_streaming=False)
                self.startup_recovery.append(index)
            addresses = tuple(self._device_address(f) for f in self.disparity_frames)
            self.disparity_write.configure(addresses, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_WIDTH)
            if getattr(self, "streaming_result", False):
                self._configure_result_reader(addresses)
            for index, channel in enumerate(self.write_channels):
                addresses = tuple(self._device_address(f) for f in self.frames[index])
                channel.configure(addresses, CAMERA_WIDTH*3, CAMERA_HEIGHT, STRIDE)
            for sensor in self.cameras:
                sensor.write(0x3008, 0x02)
            time.sleep(0.020)
            for index in range(len(self.cameras)):
                self._confirm_streaming(index)
            self.startup_pair_restart = True
            try:
                self.wait_for_frames(timeout=15.0, recover_capture_errors=True)
                return
            except TimeoutError:
                pass
        raw = self.raw_dvp_debug(timeout=0.0)
        self.startup_failure = dict(rgb_status=[hex(c.status()) for c in self.write_channels],
            rgb_errors=[c.error_names() for c in self.write_channels],
            sgm_status='0x%08x' % self.disparity_write.status(), raw_dvp=raw)
        raise RuntimeError("Startup capture validation failed after bounded recovery: "
                           "RGB=%s, RGB_errors=%s, SGM=%s, raw_DVP=%s. "
                           "A completed frame with latched capture errors is not accepted." %
                           (self.startup_failure['rgb_status'], self.startup_failure['rgb_errors'],
                            self.startup_failure['sgm_status'], raw))

    def stop(self) -> None:
        detector = getattr(self, 'detector', None)
        if detector is not None:
            detector.stop()
        committer = getattr(self, "result_committer", None)
        if committer is not None:
            committer.stop()
        for channel in (*self.write_channels, *self.read_channels, self.disparity_write, self.disparity_read):
            channel.stop()
        if getattr(self, "streaming_result", False):
            # A software stop must drain the transactional writer's issued AXI
            # burst before another Notebook construction downloads a new PL.
            self.disparity_write.reset()
        self.vtc.disable()
        self.running = False

    def wait_for_frames(self, timeout=30.0, *, recover_capture_errors=False):
        """Wait past the camera IP's power-up guard before taking snapshots."""
        if not self.running:
            raise RuntimeError("Call start() before waiting for frames")
        deadline=time.monotonic()+timeout
        pending=list(self.write_channels)+[self.disparity_write]
        retries=[0]*len(self.write_channels)
        clean_frames=[0]*len(self.write_channels)
        if recover_capture_errors and not hasattr(self, 'startup_capture_recovery'):
            self.startup_capture_recovery=[]
        while True:
            committer = getattr(self, "result_committer", None)
            if committer is not None:
                committer.check()
            statuses = [channel.status() for channel in pending]
            settling=False
            if recover_capture_errors:
                for i, channel in enumerate(self.write_channels):
                    status=statuses[i]
                    if status & VDMA_STATUS_ERROR_MASK:
                        if retries[i]<8 and channel.recover_frame_error(status):
                            retries[i]+=1;clean_frames[i]=0;settling=True
                            self.startup_capture_recovery.append(dict(channel=i,
                                status='0x%08x' % status, attempt=retries[i]))
                    elif retries[i] and status & 0x1000 and clean_frames[i]<2:
                        clean_frames[i]+=1
                        if clean_frames[i]<2:
                            channel.acknowledge_frame_complete();settling=True
            if not settling and all(status & 0x1000 and not (status & (VDMA_STATUS_ERROR_MASK | 1)) for status in statuses):
                return
            if time.monotonic()>=deadline:
                raise TimeoutError("No complete RGB/SGM frames within timeout; inspect camera.diagnostics()")
            time.sleep(.05)

    def frame(self, camera: int, *, copy: bool = True, rgb: bool = True):
        """Return one camera frame.

        Xilinx RGB video places R in the least-significant AXI byte, so VDMA's
        little-endian DDR byte order is already RGB.  The public default is an
        RGB array suitable for PIL and RGB-input neural networks.  Pass
        ``rgb=False`` when a BGR array is required by an OpenCV pipeline.
        """
        if camera not in (0, 1):
            raise ValueError("camera must be 0 or 1")
        if self.frames is None:
            raise RuntimeError("Call start() before reading frames")
        if camera >= len(self.read_channels):
            raise RuntimeError("Right RGB DDR capture was removed; frame(1) is unavailable. Both cameras still feed live SGM.")
        frame_index = self.read_channels[camera].current_frame()
        if frame_index >= len(self.frames[camera]):
            frame_index = 0
        buffer = self.frames[camera][frame_index]
        self._invalidate(buffer)
        view = buffer[:CAMERA_HEIGHT, :CAMERA_WIDTH, :]
        if not rgb:
            view = view[..., ::-1]
        return np.array(view) if copy else view

    def inference_frame(self, timeout=1.0):
        """Copy a completed left RGB frame without pausing capture or display.

        Reject snapshots overlapping a capture-buffer reuse. The bounded copy
        interval is shorter than a frame at the supported maximum sensor rate.
        """
        if not self.running: raise RuntimeError('Start camera before inference')
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            index=self.read_channels[0].current_frame()
            if index not in (0,1,2) or self.write_channels[0].current_frame()==index:
                time.sleep(.001);continue
            begin=time.monotonic();buffer=self.frames[0][index];self._invalidate(buffer)
            result=np.array(buffer[:CAMERA_HEIGHT,:CAMERA_WIDTH,:])
            if time.monotonic()-begin<1/120 and self.write_channels[0].current_frame()!=index and not self.write_channels[0].errors():
                return result
        raise TimeoutError('Could not copy a complete, non-writing left RGB buffer')

    def start_lds_net(self, threshold=.25):
        """Start autonomous PL detection and segmentation on logical left RGB."""
        if not getattr(self, 'lds_net_available', False):
            raise RuntimeError('Load the matching LDS-Net bit/HWH board package')
        if not self.running:
            raise RuntimeError('Start cameras and HDMI before LDS-Net')
        if self.detector is None:
            from .hardware_contract import validate_lds_net_reset
            validate_lds_net_reset(self._hwh_root)
            from .lds_net_backend import LDSNetPerception
            self.detector = LDSNetPerception(self._make_mmio('lds_net_perception_0'),
                                              self._make_mmio('lds_net_display_0'), threshold=threshold)
            if self.detector.bev_depth_available:
                from .sgm_snapshot import SGMSnapshot
                from .bev_geometry import BEVGeometry
                self.detector.configure_bev(SGMSnapshot(self.disparity_write.mmio,self.disparity_frames),
                                            BEVGeometry.from_directory(self.calibration_directory))
        if not np.isfinite(threshold) or not 0 <= threshold <= 1:
            raise ValueError('threshold must be within [0,1]')
        self.detector.threshold=float(threshold)
        return self.detector.start()

    def detections(self):
        if self.detector is None: return []
        return self.detector.detections()

    def bev_diagnostics(self):
        return None if self.detector is None else self.detector.bev_diagnostics()

    def perception(self, *, include_mask=False, timeout=5.0):
        if self.detector is None:
            raise RuntimeError('LDS-Net is not running')
        return self.detector.perception(include_mask=include_mask, timeout=timeout)

    def road_mask(self, *, full_frame=True, timeout=5.0):
        if self.detector is None:
            raise RuntimeError('LDS-Net is not running')
        return self.detector.road_mask(full_frame=full_frame, timeout=timeout)

    def rectified_frame(self, camera):
        """CPU preview of a rectified RGB snapshot, in disparity coordinates.

        PL rectification operates directly on live grayscale camera streams.
        This RGB preview does not feed SGM and is not its exact input frame.
        The PL Q5 grid approximates these floating maps within 0.18 pixels.
        """
        if camera not in (0, 1):
            raise ValueError("camera must be 0 or 1")
        import cv2
        if self._calibration_maps is None:
            with np.load(str(self.calibration_directory / "fixed_rig_maps.npz")) as maps:
                self._calibration_maps = {name: maps[name].copy() for name in maps.files}
        side = "left" if camera == 0 else "right"
        return cv2.remap(self.frame(camera), self._calibration_maps[side+"_x"],
                         self._calibration_maps[side+"_y"], cv2.INTER_LINEAR)

    def disparity(self, *, copy=True, visualization=False):
        """Return (480, 640) uint8 disparity in pixels (0..63).

        Set visualization=True for the RTL's 0..255 gray image used by HDMI.
        Border padding and invalid matches are not separately marked by this
        RTL. copy=False is a live CMA view only in visualization mode; integer
        conversion always creates an array.
        """
        if not self.running or self.disparity_frames is None:
            raise RuntimeError("Call start() before reading disparity")
        index = self.disparity_read.current_frame()
        if index >= len(self.disparity_frames):
            index = 0
        buffer = self.disparity_frames[index]
        self._invalidate(buffer)
        view = buffer[:CAMERA_HEIGHT, :CAMERA_WIDTH]
        if not visualization:
            # RTL: gray = {disparity[5:0], disparity[5:4]}.
            return np.right_shift(np.asarray(view), 2)
        return np.array(view) if copy else view

    def disparity_color(self):
        """Return RGB pseudocolor disparity using the HDMI fixed 0..63 palette."""
        from .palette import colorize_disparity
        return colorize_disparity(self.disparity())

    def diagnostics(self) -> dict:
        ids = []
        formats = []
        for camera in self.cameras:
            try:
                ids.append(f"0x{camera.chip_id():04x}")
                formats.append({
                    "power_0x3008": f"0x{camera.read(0x3008):02x}",
                    "pad_0x3017": f"0x{camera.read(0x3017):02x}",
                    "pad_0x3018": f"0x{camera.read(0x3018):02x}",
                    "format_0x4300": f"0x{camera.read(0x4300):02x}",
                    "mux_0x501f": f"0x{camera.read(0x501F):02x}",
                    "dvp_bits_0x3034": f"0x{camera.read(0x3034):02x}",
                    "pll_div_0x3035": f"0x{camera.read(0x3035):02x}",
                    "pll_mult_0x3036": f"0x{camera.read(0x3036):02x}",
                    "dvp_ctrl_0x460c": f"0x{camera.read(0x460C):02x}",
                })
            except Exception as exc:  # Preserve diagnostics for the remaining hardware.
                ids.append(f"ERROR: {exc}")
                formats.append(f"ERROR: {exc}")
        raw_dvp = self.raw_dvp_debug(timeout=20.0)
        line_bytes = tuple(1280 if item["rgb565_line"] else 0 for item in raw_dvp)
        return {
            "driver_build": DRIVER_BUILD,
            "lds_net_available": getattr(self, 'lds_net_available', False),
            "lds_net": self.detector.diagnostics() if getattr(self, 'detector', None) is not None else None,
            "startup_recovery_logical_cameras": getattr(self, "startup_recovery", []),
            "startup_pair_restart": getattr(self, "startup_pair_restart", False),
            "startup_capture_recovery": getattr(self, "startup_capture_recovery", []),
            "startup_failure": getattr(self, "startup_failure", None),
            "result_storage": "transactional output DDR; complete-frame publication" if getattr(self, "streaming_result", False) else "complete-frame BRAM",
            "deployment_status": self.build_manifest.get("status", "metadata unavailable"),
            "timing_closed": self.build_manifest.get("timing_closed"),
            "sgm_clock_mhz": float(self._Clocks.fclk3_mhz),
            "sgm_core_lanes": getattr(self, "sgm_core_lanes", 64),
            "sgm_core_paths": self.build_manifest.get("sgm_core_paths", 4),
            "sgm_hamming_lanes": self.build_manifest.get("sgm_hamming_lanes"),
            "sgm_path_engines": self.build_manifest.get("sgm_path_engines"),
            "sgm_wta_lanes": self.build_manifest.get("sgm_wta_lanes"),
            "sgm_output_filter": self.build_manifest.get("sgm_output_filter", "3x3 median"),
            "sgm_input_fifo_depth": self.build_manifest.get("sgm_input_fifo_depth"),
            "sgm_pixel_ii": getattr(self, "sgm_pixel_ii", 1),
            "sgm_penalties": self.build_manifest.get("sgm_penalties"),
            "sgm_max_disparity": 64,
            "sgm_output_shape": (CAMERA_HEIGHT, CAMERA_WIDTH),
            "hdmi_mode": "1920x1080p60",
            "hdmi_panel_positions": ((0, 0), (640, 0), (1280, 0)) if getattr(self, "right_rgb_available", True) else ((0, 0), (640, 0)),
            "right_rgb_available": getattr(self, "right_rgb_available", True),
            "sgm_input": "live camera AXIS (no DDR input)",
            "logical_to_physical_camera": self.physical_camera_order,
            "sgm_reference_camera": "rectified logical left / physical camera 1; original frame(0) uses unrectified coordinates",
            "stereo_rectification": "live PL: measured fixed-rig Q5 grid and bilinear interpolation",
            "stereo_rectification_installed": True,
            "camera_input_filter": getattr(self, "camera_input_filter", "unknown"),
            "disparity_palette": "Metric anchors with RGB interpolation: 1m orange, 2.5m yellow, 5m green, 10m cyan; raw disparity unchanged",
            "capture_error_names": [channel.error_names() for channel in self.write_channels],
            "disparity_error_names": self.disparity_write.error_names(),
            "disparity_write_status": f"0x{self.disparity_write.status():08x}",
            "disparity_read_status": f"0x{self.disparity_read.status():08x}",
            "disparity_write_errors": f"0x{self.disparity_write.errors():08x}",
            "disparity_read_errors": f"0x{self.disparity_read.errors():08x}",

            "sensor_ids": ids,
            "sensor_formats": formats,
            "pixel_clock_mhz": self.actual_pixel_clock_mhz,
            "read_vdma_status": [f"0x{x.status():08x}" for x in self.read_channels],
            "write_vdma_status": [f"0x{x.status():08x}" for x in self.write_channels],
            "read_vdma_errors": [f"0x{x.errors():08x}" for x in self.read_channels],
            "write_vdma_errors": [f"0x{x.errors():08x}" for x in self.write_channels],
            # The camera RTL intentionally retains the legacy power-up guard,
            # so an immediate read can still be zero.  Wait for the first
            # completed active line instead of returning a misleading value.
            "dvp_line_bytes": line_bytes,
            "raw_dvp_debug": raw_dvp,
            "expected_dvp_line_bytes": 1280,
            "rgb565_cadence_ok": tuple(value == 1280 for value in line_bytes),
            "raw_dvp_sample_x": 440,
            "raw_dvp_first_second": tuple(
                (item["first_byte"], item["second_byte"]) for item in raw_dvp
            ),
        }

    def dvp_line_bytes(self, timeout: float = 0.0):
        """Return 1280 for each camera whose last active line was RGB565."""
        return tuple(
            1280 if item["rgb565_line"] else 0
            for item in self.raw_dvp_debug(timeout=timeout)
        )

    def raw_dvp_debug(self, timeout: float = 0.0):
        """Read both raw RGB565 DVP bytes sampled at x=440 before packing."""
        deadline = time.monotonic() + timeout
        while True:
            packed = self._mmios["cmos_rst"].read(0x08)
            physical_words = (packed & 0xFFFF, (packed >> 16) & 0xFFFF)
            words = tuple(physical_words[i] for i in self.physical_camera_order)
            result = tuple({
                "valid": bool(word & 0x8000),
                "rgb565_line": bool(word & 0x4000),
                # The hardware stores bits 7:1. Expand back to an even 8-bit
                # approximation; the discarded LSB cannot affect diagnosis.
                "first_byte": ((word >> 7) & 0x7F) << 1,
                "second_byte": (word & 0x7F) << 1,
            } for word in words)
            if all(item["valid"] for item in result) or time.monotonic() >= deadline:
                return result
            time.sleep(0.05)

    def close(self) -> None:
        detector=getattr(self,'detector',None)
        if detector is not None:
            detector.close();self.detector=None
        if self.running:
            self.stop()
        for camera in self.cameras:
            camera.close()
        if self.frames is not None:
            for camera_frames in self.frames:
                for frame in camera_frames:
                    if hasattr(frame, "close"):
                        frame.close()
                    elif hasattr(frame, "freebuffer"):
                        frame.freebuffer()
            self.frames = None

        if self.disparity_frames is not None:
            for frame in self.disparity_frames:
                if hasattr(frame, "close"):
                    frame.close()
                elif hasattr(frame, "freebuffer"):
                    frame.freebuffer()
            self.disparity_frames = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_exc):
        self.close()


# Compatibility name: both APIs load the same complete SGM-YOLO hardware.
DualOV5640Overlay = SGMYOLOOverlay
