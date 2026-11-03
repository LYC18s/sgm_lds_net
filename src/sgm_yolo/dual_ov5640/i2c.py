"""Small Linux I2C helper with no third-party Python dependency."""

import errno
import os
import time
from pathlib import Path


I2C_SLAVE = 0x0703
_TRANSIENT_ERRORS = {errno.EAGAIN, errno.EBUSY, errno.EINTR}
_IO_RETRIES = 8


class LinuxI2C:
    """Access one 7-bit I2C slave through a Linux ``/dev/i2c-*`` node."""

    def __init__(self, bus, address):
        self.path = Path(bus) if isinstance(bus, (str, Path)) else Path(f"/dev/i2c-{bus}")
        self.address = address
        self._fd = None

    def open(self):
        if self._fd is None:
            try:
                import fcntl
            except ImportError as exc:
                raise RuntimeError("Linux I2C access is only available on the PYNQ board") from exc
            try:
                self._fd = os.open(self.path, os.O_RDWR)
            except OSError as exc:
                raise RuntimeError(
                    f"Cannot open {self.path}. Check the PYNQ device tree and I2C bus numbering."
                ) from exc
            fcntl.ioctl(self._fd, I2C_SLAVE, self.address)
        return self

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> "LinuxI2C":
        return self.open()

    def __exit__(self, *_exc) -> None:
        self.close()

    def write_reg16(self, register: int, value: int) -> None:
        self.open()
        payload = bytes(((register >> 8) & 0xFF, register & 0xFF, value & 0xFF))
        written = self._retry(lambda: os.write(self._fd, payload))
        if written != len(payload):
            raise IOError(f"Short I2C write to 0x{self.address:02x}: {written}/{len(payload)}")

    def read_reg16(self, register: int) -> int:
        self.open()
        address = bytes(((register >> 8) & 0xFF, register & 0xFF))
        if self._retry(lambda: os.write(self._fd, address)) != 2:
            raise IOError(f"Short I2C register-address write to 0x{self.address:02x}")
        data = self._retry(lambda: os.read(self._fd, 1))
        if len(data) != 1:
            raise IOError(f"Short I2C read from 0x{self.address:02x}")
        return data[0]

    @staticmethod
    def _retry(operation):
        """Retry transient Linux I2C busy/interrupted errors with backoff."""
        for attempt in range(_IO_RETRIES):
            try:
                return operation()
            except OSError as exc:
                if exc.errno not in _TRANSIENT_ERRORS or attempt == _IO_RETRIES - 1:
                    raise
                time.sleep(0.005 * (attempt + 1))
