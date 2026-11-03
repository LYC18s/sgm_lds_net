"""Register-level Video Timing Controller generator setup."""

from collections import namedtuple


VideoMode = namedtuple(
    "VideoMode",
    "width height h_front h_sync h_back v_front v_sync v_back "
    "h_positive v_positive pixel_clock_mhz",
)


MODE_1080P60 = VideoMode(1920, 1080, 88, 44, 148, 4, 5, 36, True, True, 148.5)
MODE_1080P30 = MODE_1080P60._replace(pixel_clock_mhz=74.25)


class VideoTimingController:
    # Source-selection bits used by the original XVtc_SetSource call.
    _GENERATOR_SOURCES = 0x03F7EF00

    def __init__(self, mmio):
        self.mmio = mmio

    @staticmethod
    def _pair(start: int, end: int) -> int:
        return (start & 0x1FFF) | ((end & 0x1FFF) << 16)

    def disable(self) -> None:
        self.mmio.write(0x00, self.mmio.read(0x00) & ~0x4)

    def enable(self) -> None:
        self.mmio.write(0x00, self.mmio.read(0x00) | 0x4)

    def configure(self, mode: VideoMode = MODE_1080P60, *, enable=True) -> None:
        h_sync_start = mode.width + mode.h_front
        h_back_start = h_sync_start + mode.h_sync
        h_total = h_back_start + mode.h_back
        v_sync_start = mode.height + mode.v_front - 1
        v_back_start = v_sync_start + mode.v_sync
        v_total = v_back_start + mode.v_back + 1

        polarity = 0x70  # active chroma, active video and field ID
        if mode.v_positive:
            polarity |= 0x05  # vblank and vsync
        if mode.h_positive:
            polarity |= 0x0A  # hblank and hsync

        control = self.mmio.read(0x00)
        self.mmio.write(0x00, control | 0x2)  # register update enable
        self.mmio.write(0x6C, polarity)
        self.mmio.write(0x70, h_total)
        self.mmio.write(0x74, self._pair(v_total, v_total))
        self.mmio.write(0x60, self._pair(mode.width, mode.height))
        self.mmio.write(0x78, self._pair(h_sync_start, h_back_start))
        self.mmio.write(0x80, self._pair(v_sync_start, v_back_start))
        self.mmio.write(0x8C, self._pair(v_sync_start, v_back_start))
        self.mmio.write(0x7C, self._pair(mode.width, mode.width))
        self.mmio.write(0x84, self._pair(h_sync_start, h_sync_start))
        self.mmio.write(0x88, self._pair(mode.width, mode.width))
        self.mmio.write(0x90, self._pair(h_sync_start, h_sync_start))
        encoding = self.mmio.read(0x68) & ~(0x100 | 0x40)
        self.mmio.write(0x68, encoding)
        control = self.mmio.read(0x00) & ~0x03FDEF00
        control = (control | self._GENERATOR_SOURCES | 0x2) & ~0x4
        self.mmio.write(0x00, control | (0x4 if enable else 0))
