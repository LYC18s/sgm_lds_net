"""Driver for the Digilent ``axi_dynclk`` instance in the legacy design."""

import math
import time


# Values from the original BSD-licensed Digilent dynclk driver bundled with
# this project. Index is CLKFBOUT_MULT_F - 1.
LOCK_LOOKUP = (
    0x31BE8FA401, 0x31BE8FA401, 0x423E8FA401, 0x5AFE8FA401,
    0x73BE8FA401, 0x8C7E8FA401, 0x9CFE8FA401, 0xB5BE8FA401,
    0xCE7E8FA401, 0xE73E8FA401, 0xFFF84FA401, 0xFFF39FA401,
    0xFFEEEFA401, 0xFFEBCFA401, 0xFFE8AFA401, 0xFFE71FA401,
    0xFFE3FFA401, 0xFFE26FA401, 0xFFE0DFA401, 0xFFDF4FA401,
    0xFFDDBFA401, 0xFFDC2FA401, 0xFFDA9FA401, 0xFFD90FA401,
    0xFFD90FA401, 0xFFD77FA401, 0xFFD5EFA401, 0xFFD5EFA401,
    0xFFD45FA401, 0xFFD45FA401, 0xFFD2CFA401, 0xFFD2CFA401,
    0xFFD2CFA401, 0xFFD13FA401, 0xFFD13FA401, 0xFFD13FA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
    0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401, 0xFFCFAFA401,
)

FILTER_LOOKUP_LOW = (
    0x05F, 0x057, 0x07B, 0x05B, 0x06B, 0x073, 0x073, 0x073,
    0x073, 0x04B, 0x04B, 0x04B, 0x0B3, 0x053, 0x053, 0x053,
    0x053, 0x053, 0x053, 0x053, 0x053, 0x053, 0x053, 0x063,
    0x063, 0x063, 0x063, 0x063, 0x063, 0x063, 0x063, 0x063,
    0x063, 0x063, 0x063, 0x063, 0x063, 0x093, 0x093, 0x093,
    0x093, 0x093, 0x093, 0x093, 0x093, 0x093, 0x093, 0x0A3,
    0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3,
    0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3, 0x0A3,
)


def _divider(divide: int) -> int:
    if not 1 <= divide <= 128:
        raise ValueError(f"Invalid MMCM divider {divide}")
    if divide == 1:
        return 0x1041
    high = divide // 2
    low = high + (divide & 1)
    return ((divide & 1) << 13) | (low & 0x3F) | ((high << 6) & 0xFC0)


def _count(divide: int) -> int:
    value = _divider(divide)
    return (value & 0xFFF) | ((value << 10) & 0x00C00000)


class DynamicClock:
    def __init__(self, mmio, reference_mhz: float = 100.0):
        self.mmio = mmio
        self.reference_mhz = reference_mhz

    def _wait_running(self, expected: bool, timeout: float = 1.0) -> None:
        deadline = time.monotonic() + timeout
        while bool(self.mmio.read(0x04) & 1) != expected:
            if time.monotonic() >= deadline:
                state = "start" if expected else "stop"
                raise TimeoutError(f"axi_dynclk did not {state} within {timeout:.1f}s")
            time.sleep(0.001)

    def stop(self) -> None:
        self.mmio.write(0x00, 0)
        self._wait_running(False)

    def configure(self, pixel_clock_mhz: float) -> float:
        target = pixel_clock_mhz * 5.0
        best = None
        for main_div in range(1, 11):
            min_fb = main_div * 6
            max_fb = min(main_div * 12, 64)
            multiplier = (self.reference_mhz / main_div) / target
            for fb_mult in range(min_fb, max_fb + 1):
                clk_div = math.floor(multiplier * fb_mult + 0.5)
                if not 1 <= clk_div <= 128:
                    continue
                actual = (self.reference_mhz / main_div / clk_div) * fb_mult
                candidate = (abs(actual - target), main_div, fb_mult, clk_div, actual / 5.0)
                if best is None or candidate[0] < best[0]:
                    best = candidate
        if best is None:
            raise ValueError(f"Cannot synthesize {pixel_clock_mhz} MHz")

        _, main_div, fb_mult, clk_div, actual_mhz = best
        lock = LOCK_LOOKUP[fb_mult - 1]
        self.mmio.write(0x08, _count(clk_div))
        self.mmio.write(0x0C, _count(fb_mult))
        self.mmio.write(0x10, 0)
        self.mmio.write(0x14, _divider(main_div))
        self.mmio.write(0x18, lock & 0xFFFFFFFF)
        self.mmio.write(0x1C, ((lock >> 32) & 0xFF) | ((FILTER_LOOKUP_LOW[fb_mult - 1] << 16) & 0x03FF0000))
        self.stop()
        self.mmio.write(0x00, 1)
        self._wait_running(True)
        return actual_mhz
