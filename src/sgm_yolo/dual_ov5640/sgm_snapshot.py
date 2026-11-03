"""Token-owned next-output-SOF SGM snapshots; compatible with display ACKs."""
import threading
import time
import numpy as np

SNAPSHOT_CAPABILITY=0x534e5031


class SnapshotCancelled(RuntimeError):
    pass


class SGMSnapshot:
    @staticmethod
    def require_capability(mmio):
        actual=int(mmio.read(0xf0))
        if actual!=SNAPSHOT_CAPABILITY:
            raise RuntimeError('SGM SNP1 frame lease hardware is required: '
                'register 0xF0=0x%08x, expected 0x%08x. Install the matching SNP1 bit/HWH; '
                'a driver-only update cannot add this hardware interface.' % (actual,SNAPSHOT_CAPABILITY))
        return actual

    def __init__(self,mmio,frames):
        self.require_capability(mmio)
        self.mmio,self.frames=mmio,frames
        self._lock=threading.Lock();self._token=int(mmio.read(0x10c))

    def _valid(self,token,sequence=None):
        status=self.mmio.read(0xfc)
        return (status&0x1b)==2 and self.mmio.read(0x10c)==token and (
            sequence is None or self.mmio.read(0x104)==sequence)

    @staticmethod
    def _invalidate(frame,windows,radius):
        if getattr(frame,'coherent',False):return
        if hasattr(frame,'device') and hasattr(frame,'bo'):
            spans=set()
            for center in windows:
                if center is None:continue
                x,y=center
                for row in range(y-radius,y+radius+1):
                    lo=(row*640+x-radius)//64*64
                    hi=((row*640+x+radius+1)+63)//64*64
                    spans.add((lo,hi))
            for lo,hi in sorted(spans):
                frame.device.invalidate(frame.bo,int(getattr(frame,'offset',0))+lo,
                    int(frame.ctypes.data)+lo,hi-lo)
        elif hasattr(frame,'invalidate'):
            # Older Xlnk versions: invalidate cache only, never copy full frame.
            frame.invalidate()

    def sample_next(self,points,*,radius=2,stop=None,timeout=.75):
        if radius not in (1,2):raise ValueError('Use a 3x3 or 5x5 window')
        if len(points)>320:raise ValueError('At most 320 centers')
        with self._lock:
            self._token=(self._token+1)&0xffffffff or 1;token=self._token
            if self.mmio.read(0xfc)&3:raise RuntimeError('Previous SGM lease is still owned')
            windows=[]
            for point in points:
                if point is None or not np.all(np.isfinite(point)):
                    windows.append(None);continue
                x,y=(int(round(float(v))) for v in point)
                windows.append((x,y) if radius<=x<640-radius and radius<=y<480-radius else None)
            self.mmio.write(0xf8,token);self.mmio.write(0xf4,1)
            deadline=time.monotonic()+timeout
            try:
                while not self._valid(token):
                    if stop is not None and stop.is_set():raise SnapshotCancelled('Snapshot cancelled')
                    if self.mmio.read(0xfc)&0x18:raise RuntimeError('SGM snapshot expired or rejected')
                    if time.monotonic()>=deadline:raise TimeoutError('No next complete SGM frame')
                    time.sleep(.001)
                index=int(self.mmio.read(0x100));sequence=int(self.mmio.read(0x104))
                if index not in (0,1,2):raise RuntimeError('Invalid snapshot buffer')
                frame=self.frames[index]
                if np.shape(frame)!=(480,640):raise RuntimeError('Unexpected SGM buffer shape')
                self._invalidate(frame,windows,radius)
                patches=[None if center is None else np.array(frame[
                    center[1]-radius:center[1]+radius+1,center[0]-radius:center[0]+radius+1],copy=True)>>2
                    for center in windows]
                if not self._valid(token,sequence):raise RuntimeError('SGM lease lost during sampling')
                return dict(request_id=token,sgm_sequence=sequence,buffer_index=index,
                    centers=windows,patches=patches)
            finally:
                self.mmio.write(0x114,token)
