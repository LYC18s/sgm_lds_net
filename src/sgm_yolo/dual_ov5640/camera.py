"""OV5640 control used by the two AN5642 sensors."""

import time

from .i2c import LinuxI2C
from .ov5640_registers import OV5640_VGA_RGB565_REGISTERS


OV5640_I2C_ADDRESS = 0x3C


class OV5640:
    def __init__(self, bus, address=OV5640_I2C_ADDRESS):
        self.i2c = LinuxI2C(bus, address)

    def close(self) -> None:
        self.i2c.close()

    def read(self, register: int) -> int:
        return self.i2c.read_reg16(register)

    def write(self, register: int, value: int) -> None:
        self.i2c.write_reg16(register, value)

    def chip_id(self) -> int:
        return (self.read(0x300A) << 8) | self.read(0x300B)

    def initialize_vga_rgb565(self, verify_id: bool = True, *, start_streaming: bool = True) -> None:
        if verify_id:
            chip_id = self.chip_id()
            if chip_id != 0x5640:
                raise RuntimeError(f"Expected OV5640 on {self.i2c.path}, found chip ID 0x{chip_id:04x}")
        for register, value in OV5640_VGA_RGB565_REGISTERS:
            if register == 0x3008 and value == 0x02 and not start_streaming:
                continue
            self.write(register, value)
            if register == 0x3008 and value == 0x82:
                time.sleep(0.005)
            elif register == 0x3008 and value == 0x02:
                # Let the sensor leave standby before the read-back check.
                time.sleep(0.020)
            elif register in (0x4300, 0x501F):
                # The reference driver allows the ISP/DVP format mux to settle
                # after each format write.
                time.sleep(0.001)

        expected = {
            0x3808: 0x02, 0x3809: 0x80,
            0x380A: 0x01, 0x380B: 0xE0,
            0x4300: 0x60, 0x501F: 0x01,
            # AN5642 board-specific clock profile.  Replacing this complete
            # group with the generic ST DVP profile changes the active line
            # length and prevents VDMA capture on this carrier.
            0x3034: 0x1A, 0x3035: 0x21, 0x3036: 0x69,
            0x3037: 0x13, 0x3108: 0x01, 0x460C: 0x20,
            0x3008: 0x02 if start_streaming else 0x42,
        }
        # Keep one actual value per attempt and retry a transient bad sample.
        # The old dict comprehension read mismatches twice, which could report
        # the contradictory message "expected X, got X".
        mismatches = {}
        for register, value in expected.items():
            for attempt in range(3):
                actual = self.read(register)
                if actual == value:
                    break
                if attempt < 2:
                    time.sleep(0.002)
            if actual != value:
                mismatches[register] = (value, actual)
        if mismatches:
            detail = ", ".join(
                f"0x{register:04x}: expected 0x{wanted:02x}, got 0x{actual:02x}"
                for register, (wanted, actual) in mismatches.items()
            )
            raise RuntimeError(f"OV5640 VGA/RGB565 register verification failed: {detail}")
