"""Latest-only PS UI worker and frame-boundary, drain-acknowledged CMA ownership."""
import threading
import time
import numpy as np
from .ui_render import UIRenderer,UI_BYTES

UI_CAPABILITY=0x55494631


def pack_hdmi_bgr_inplace(buffer,scratch=None):
    """RGB canvas -> little-endian bytes for the PL/HDMI 0xRRGGBB word.

    The row fetcher preserves DDR byte order and places byte 0 in bits 7:0,
    which rgb2dvi sends as blue. Keep RGB rendering/color estimation intact;
    convert only the completed UI atlas, before its cache flush/publication.
    One reusable channel scratch avoids a full-frame temporary per update.
    """
    data=np.asarray(buffer)
    if data.dtype!=np.uint8 or not data.flags.c_contiguous or data.size<UI_BYTES:
        raise ValueError('UI packing requires contiguous uint8 atlas storage')
    pixels=data.reshape(-1)[:UI_BYTES].reshape(-1,3)
    if scratch is None:scratch=np.empty(UI_BYTES//3,np.uint8)
    if scratch.shape!=(UI_BYTES//3,) or scratch.dtype!=np.uint8:
        raise ValueError('Invalid UI channel scratch')
    np.copyto(scratch,pixels[:,0]);pixels[:,0]=pixels[:,2];pixels[:,2]=scratch
    return scratch


class UIFramebuffer:
    _retained=[]
    def __init__(self,display,allocator,cache,io_lock,renderer=None,timeout=.5,max_update_hz=20.):
        if display.read(0x300)!=UI_CAPABILITY:raise RuntimeError('PS UI framebuffer ABI mismatch')
        self.display,self.allocate,self.cache,self.io=display,allocator,cache,io_lock
        self.renderer=renderer;self.timeout=timeout
        self.max_update_hz=float(max_update_hz)
        if not 0<self.max_update_hz<=60:raise ValueError('UI update limit must be within (0,60] Hz')
        self.buffers=[];self.front=None;self.sequence=int(display.read(0x31c));self.poisoned=False
        self.condition=threading.Condition();self.stopping=False;self.job=None;self.thread=None
        self.scene=None;self.pedestrian=None
        self.error=None;self.count=0;self.dropped=0;self.render_ms=0.;self.latest=None
        self._color_scratch=None

    @staticmethod
    def _address(buffer):return int(buffer.device_address if hasattr(buffer,'device_address') else buffer.physical_address)

    def _wait(self,sequence):
        deadline=time.monotonic()+self.timeout
        while self.display.read(0x304)&1 or self.display.read(0x31c)!=sequence:
            if time.monotonic()>deadline:
                self.poisoned=True
                raise TimeoutError('UI reader did not drain/acknowledge; CMA retained')
            time.sleep(.001)

    def _commit(self,address,frame,sgm,enabled=True):
        self.sequence=(self.sequence+1)&0xffffffff
        with self.io:
            self.display.write(0x308,address);self.display.write(0x30c,self.sequence)
            self.display.write(0x310,int(frame)&0xffffffff);self.display.write(0x314,int(sgm)&0xffffffff)
            self.display.write(0x318,1 if enabled else 2)
        self._wait(self.sequence)

    def start(self):
        if self.thread is not None and self.thread.is_alive():return
        if self.poisoned:raise RuntimeError('UI reader owns retained CMA after a timeout')
        if self.renderer is None:self.renderer=UIRenderer()
        # Retire a prior reader before allocating/reusing any UI memory.
        self._commit(0,0,0,False)
        try:
            for _ in range(3):
                b=self.allocate(UI_BYTES);self.buffers.append(b)
                a=self._address(b)
                if b.nbytes<UI_BYTES or a<=0 or a%64 or a+UI_BYTES>2**32:
                    raise ValueError('UI requires 64-byte aligned, contiguous 32-bit CMA')
            ranges=sorted((self._address(b),self._address(b)+UI_BYTES) for b in self.buffers)
            if any(x[1]>y[0] for x,y in zip(ranges,ranges[1:])):raise ValueError('UI buffers overlap')
        except Exception:
            self._free();raise
        self.stopping=False;self.error=None
        self.thread=threading.Thread(target=self._run,name='PS-UI',daemon=True);self.thread.start()

    def submit(self,frame,sgm,words,flags=0):
        with self.condition:
            if self.stopping or self.poisoned:return
            if self.job is not None:self.dropped+=1
            self.job=(int(frame),int(sgm),tuple(words),int(flags),time.monotonic())
            self.scene=self.job
            self.condition.notify_all()

    def set_pedestrian(self,value):
        # Preserve the original direct NN pedestrian hint, independent of SGM.
        with self.condition:
            if self.stopping or self.poisoned:return
            self.pedestrian=(bool(value),time.monotonic())
            if self.job is None:self.job=self.scene
            self.condition.notify_all()

    def _run(self):
        last=None;expired=False;dimmed=False;published_at=0.;next_render_at=0.
        while True:
            with self.condition:
                if self.job is None and not self.stopping:self.condition.wait(.05)
                if self.stopping:return
                remaining=next_render_at-time.monotonic()
                if remaining>0:self.condition.wait(remaining);continue
                job=self.job;self.job=None
            if job is None:
                if last is None:continue
                age=time.monotonic()-last[4]
                if age>=3 and not dimmed:job=(last[0],last[1],(0,)*16,0,last[4]);dimmed=True
                elif age>=1 and not expired:job=(last[0],last[1],(0,)*16,last[3],last[4]);expired=True
                elif age>=1 and time.monotonic()-published_at>=.5:
                    job=(last[0],last[1],(0,)*16,0 if age>=3 else last[3],last[4])
                else:continue
            else:last=job;expired=False;dimmed=False
            next_render_at=time.monotonic()+1/self.max_update_hz
            try:
                index=next(i for i in range(3) if i!=self.front);buffer=self.buffers[index]
                begin=time.monotonic();age=begin-job[4]
                words=job[2] if age<1 else (0,)*16
                flags=job[3] if age<3 else 0
                pedestrian=self.pedestrian
                if pedestrian is not None and begin-pedestrian[1]<3:flags=(flags&~1)|int(pedestrian[0])
                self.renderer.render(buffer,words,flags)
                self._color_scratch=pack_hdmi_bgr_inplace(buffer,self._color_scratch)
                self.cache(buffer,UI_BYTES,'flush')
                self.render_ms=(time.monotonic()-begin)*1000
                self._commit(self._address(buffer),job[0],job[1]);self.front=index
                published_at=time.monotonic()
                self.count+=1;self.latest=dict(detection_frame=job[0],sgm_sequence=job[1],sequence=self.sequence)
                self.error=None
            except Exception as error:
                self.error=str(error)
                if self.poisoned:return

    def _free(self):
        for b in self.buffers:
            if hasattr(b,'freebuffer'):b.freebuffer()
            elif hasattr(b,'close'):b.close()
        self.buffers=[];self.front=None

    def stop(self,timeout=2.):
        with self.condition:self.stopping=True;self.job=None;self.condition.notify_all()
        if self.thread is not None:self.thread.join(timeout)
        try:
            if self.thread is not None and self.thread.is_alive():raise TimeoutError('UI worker still running')
            if self.poisoned:raise TimeoutError('UI DMA retirement not proven')
            self._commit(0,0,0,False);self._free()
        except Exception as error:
            self.error=str(error);self.poisoned=True
            if self.buffers:self._retained.append(self.buffers);self.buffers=[]
        self.thread=None

    def diagnostics(self):
        return dict(mode='ps_bgr888_triple_buffer',memory_byte_order='BGR',video_word='0xRRGGBB',updates=self.count,dropped=self.dropped,
            render_ms=self.render_ms,max_update_hz=self.max_update_hz,error=self.error,retained=self.poisoned,latest=self.latest,
            underruns=self.display.read(0x320),read_errors=self.display.read(0x324))
