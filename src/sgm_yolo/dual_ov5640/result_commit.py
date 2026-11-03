"""Publish completed DDR result buffers without exposing a partial SGM frame."""
import threading


class ResultCommitter:
    _active = set()

    @classmethod
    def stop_existing(cls):
        # Re-running a Notebook's construction cell must stop old publishers
        # before the new Overlay replaces the PL or the old buffers are freed.
        for committer in tuple(cls._active):
            if committer.on_replace is not None:
                committer.on_replace()
            else:
                committer.stop()

    def __init__(self, writer_mmio, reader, on_replace=None):
        self.mmio = writer_mmio
        self.reader = reader
        self.on_replace = on_replace
        self._stop = threading.Event()
        self.thread = None
        self.error = None

    def poll(self):
        published = self.mmio.read(0xE0)
        if not published & 0x80000000:
            return False
        sequence = self.mmio.read(0xE4)
        if self.mmio.read(0xE8) == sequence:
            return False
        index = published & 3
        if index >= 3:
            raise RuntimeError("Invalid committed disparity buffer")
        self.reader.park_frame(index)
        # Requesting a buffer is not an acknowledgement. Wait until the read
        # DMA reports that this buffer is actually active at a frame boundary.
        if self.reader.current_frame() == index:
            self.mmio.write(0xE8, sequence)
            return True
        return False

    def _run(self):
        try:
            while not self._stop.is_set():
                self.poll()
                self._stop.wait(0.002)
        except Exception as exc:
            self.error = exc

    def check(self):
        if self.error is not None:
            raise RuntimeError("Disparity frame publication failed") from self.error

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self._stop.clear()
        self.error = None
        self.thread = threading.Thread(target=self._run, name="sgm-frame-commit", daemon=True)
        self._active.add(self)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                raise RuntimeError("Disparity frame publication thread did not stop")
            self.thread = None
        self._active.discard(self)
