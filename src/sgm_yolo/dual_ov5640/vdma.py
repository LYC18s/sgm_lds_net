"""Minimal direct-register AXI VDMA driver for the four single-channel VDMAs."""

import time


MM2S_CONTROL = 0x00
MM2S_STATUS = 0x04
PARK_POINTER = 0x28
FRAME_POINTER_STATUS = PARK_POINTER
S2MM_CONTROL = 0x30
S2MM_STATUS = 0x34
MM2S_VSIZE = 0x50
MM2S_HSIZE = 0x54
MM2S_STRIDE = 0x58
MM2S_START = 0x5C
S2MM_VSIZE = 0xA0
S2MM_HSIZE = 0xA4
S2MM_STRIDE = 0xA8
S2MM_START = 0xAC

# PG020: error flags are bits 4..8, 11 and 15; bits 9..10 are reserved.
# Bits 12..14 are
# normal frame-count, delay-count, and error-summary interrupt flags; in
# particular 0x1000 means a frame completed and must not be reported as an
# error.
VDMA_STATUS_ERROR_MASK = 0x000089F0
VDMA_ERROR_FLAGS = (
    (0x010, "internal"), (0x020, "slave"), (0x040, "decode"),
    (0x080, "frame_too_short"), (0x100, "line_too_short"),
    (0x800, "frame_too_long"), (0x8000, "line_too_long"),
)


class VDMAChannel:
    def __init__(self, mmio, direction: str, *, flush_on_fsync=False):
        if direction not in {"read", "write"}:
            raise ValueError("direction must be 'read' (MM2S) or 'write' (S2MM)")
        self.mmio = mmio
        self.direction = direction
        self.flush_on_fsync = bool(flush_on_fsync)

    @property
    def _control(self) -> int:
        return MM2S_CONTROL if self.direction == "read" else S2MM_CONTROL

    @property
    def _status(self) -> int:
        return MM2S_STATUS if self.direction == "read" else S2MM_STATUS

    def reset(self, timeout: float = 1.0) -> None:
        self.mmio.write(self._control, 0x4)
        deadline = time.monotonic() + timeout
        while self.mmio.read(self._control) & 0x4:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"VDMA {self.direction} reset timed out")
            time.sleep(0.001)

    def stop(self) -> None:
        self.mmio.write(self._control, self.mmio.read(self._control) & ~1)

    def configure(self, addresses, hsize: int, vsize: int, stride: int, *, parked=False, frame_index=0) -> None:
        if isinstance(addresses, int):
            addresses = (addresses,)
        else:
            addresses = tuple(addresses)
        if not addresses:
            raise ValueError("At least one frame-buffer address is required")
        if len(addresses) > 3:
            raise ValueError("This overlay is built with exactly three VDMA frame stores")
        if any(address >= 1 << 32 for address in addresses):
            raise ValueError("This Zynq-7000 VDMA design only supports 32-bit buffer addresses")
        if not (0 < hsize <= stride):
            raise ValueError("hsize must be positive and no larger than stride")
        self.reset()
        if self.direction == "read":
            # Circular mode + Genlock slave. A one-frame delay keeps HDMI on a
            # completed buffer while capture writes the next one.
            self.mmio.write(MM2S_CONTROL, 0x1 if parked else 0xB)
            for index, address in enumerate(addresses):
                self.mmio.write(MM2S_START + 4 * index, address)
            self.mmio.write(MM2S_STRIDE, stride if parked else (1 << 24) | stride)
            self.mmio.write(MM2S_HSIZE, hsize)
            if parked:
                self.park_frame(frame_index)
            self.mmio.write(MM2S_VSIZE, vsize)  # VSIZE must be written last.
        else:
            # Circular Genlock master cycles through three capture buffers.
            self.mmio.write(S2MM_CONTROL, 0x3)
            for index, address in enumerate(addresses):
                self.mmio.write(S2MM_START + 4 * index, address)
            self.mmio.write(S2MM_STRIDE, stride)
            self.mmio.write(S2MM_HSIZE, hsize)
            self.mmio.write(S2MM_VSIZE, vsize)  # VSIZE must be written last.

    def status(self) -> int:
        return self.mmio.read(self._status)

    def park_frame(self, index):
        if self.direction != "read" or index not in (0, 1, 2):
            raise ValueError("Only a read channel can park on frame 0, 1 or 2")
        self.mmio.write(PARK_POINTER, (self.mmio.read(PARK_POINTER) & ~0x1F) | index)

    def errors(self) -> int:
        return self.status() & VDMA_STATUS_ERROR_MASK

    def error_names(self):
        errors = self.errors()
        return [name for mask, name in VDMA_ERROR_FLAGS if errors & mask]

    def clear_errors(self):
        """Acknowledge writable error/IRQ flags; a persistent fault can relatch.

        Some frame flags are read-only without flush-on-fsync. This method
        does not promise recovery from a halted channel; reset is then needed.
        """
        self.mmio.write(self._status, self.errors() | 0x4000)

    def recover_frame_error(self, status):
        """Acknowledge a running S2MM's recoverable frame/line error.

        Flush-on-fsync hardware resynchronizes on a subsequent SOF. Clear the
        old completion IRQ as well: callers must observe NEW clean completions
        before claiming recovery. Bus faults, halted DMA and standalone internal
        faults require reset/diagnosis and are never acknowledged here.
        """
        errors = status & VDMA_STATUS_ERROR_MASK
        geometry = errors & 0x8980
        if (self.direction != 'write' or not self.flush_on_fsync or status & 1
                or errors & 0x60 or not geometry
                or (errors & 0x10 and not errors & 0x880)
                or not self.mmio.read(self._control) & 1):
            return False
        self.mmio.write(self._status, errors | 0x5000)
        return True

    def acknowledge_frame_complete(self):
        self.mmio.write(self._status, 0x1000)

    def current_frame(self) -> int:
        status = self.mmio.read(FRAME_POINTER_STATUS)
        if self.direction == "read":
            return (status >> 16) & 0x1F
        return (status >> 24) & 0x1F
