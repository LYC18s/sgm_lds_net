"""Whole-frame LDS-Net board backend; no PS intermediate tensor operations.

Normal processing invalidates/copies only the7000 detection bytes. Optional
mask snapshots are copied while PS owns a completed slot, before publication;
reading a mask from an asynchronously recycled active DDR slot is forbidden.
"""
from collections import deque
import math
from pathlib import Path
import threading
import time
import weakref
import json
import numpy as np
from .lds_net import NativeDecoder,unpack_mask,validate_package,CROP
from .alerts import HUD_CAPABILITY,AlertLatch
from .bev import BEV_CAPABILITY,BEV_BASE,descriptors
from .bev_fusion import BEV2_CAPABILITY,BEVFusion
from .bev_color import RGB_CAPABILITY,COLOR_CAPABILITY,attach_colors
from .ui_framebuffer import UI_CAPABILITY,UIFramebuffer

HARDWARE_ABI=0x524C0003
DISPLAY_ABI=0x524C4401
CAPABILITIES=0x1f
SIZES=(307200,307200,614400,376384,26240,26240,57600,57600)


def _address(buffer):
    return int(buffer.device_address if hasattr(buffer,'device_address') else buffer.physical_address)


def _cache(buffer,size,operation):
    size=min(int(buffer.nbytes),(int(size)+63)//64*64)
    # PYNQ 2.5 views do not inherit device/bo. Use the allocation's device
    # directly, retaining the bounded cache range without constructing a view.
    if getattr(buffer,'coherent',False):
        return
    if hasattr(buffer,'device') and hasattr(buffer,'bo'):
        getattr(buffer.device,operation)(buffer.bo,int(getattr(buffer,'offset',0)),
                                        int(buffer.ctypes.data),size)
    elif hasattr(buffer,'device_address'):
        getattr(buffer[:size],operation)()
    elif getattr(buffer,'cacheable',False) and hasattr(buffer,'pointer'):
        from pynq import Xlnk
        function=Xlnk.libxlnk.cma_flush_cache if operation=='flush' else Xlnk.libxlnk.cma_invalidate_cache
        function(buffer.pointer,_address(buffer),size)
    elif hasattr(buffer,operation):
        getattr(buffer,operation)()


def _allocate(size):
    try:
        from pynq import allocate
        return allocate(shape=(size,),dtype=np.uint8)
    except ImportError:
        from pynq import Xlnk
        return Xlnk().cma_array(shape=(size,),dtype=np.uint8,cacheable=1)


class LDSNetPerception:
    _instances=weakref.WeakSet()
    # Retain CMA after a timeout. An object/array finalizer must not silently
    # free memory that a still-running PL master could access.
    _retained=[]

    def __init__(self,mmio,display,*,model_directory=None,threshold=.25,allocator=None,decoder=None,timeout=5.0):
        if not math.isfinite(threshold) or not 0<=threshold<=1:
            raise ValueError('threshold must be within[0,1]')
        self.mmio,self.display=mmio,display
        self.root=Path(model_directory or Path(__file__).resolve().parents[1]/'models/lds_net')
        self.contract=validate_package(self.root)
        if mmio.read(0)!=HARDWARE_ABI or mmio.read(12)&CAPABILITIES!=CAPABILITIES or display.read(0)!=DISPLAY_ABI:
            raise RuntimeError('Load matching LDS-Net autonomous bit/HWH and driver (ABI524c0003/524c4401)')
        hardware_hash=sum(int(mmio.read(0x80+i*4))<<(32*i) for i in range(8))
        if hardware_hash!=int(self.contract['network_sha256'],16):
            raise RuntimeError('LDS-Net PL model hash does not match the deployment package')
        if [mmio.read(a) for a in (0xa0,0xa4,0xa8,0xac)]!=[307200,614400,26200,57600]:
            raise RuntimeError('LDS-Net PL tensor sizes do not match the driver')
        self.decoder=decoder if decoder is not None else NativeDecoder(json.loads((self.root/'network.json').read_text(encoding='utf-8')))
        self.threshold=float(threshold);self.timeout=float(timeout);self._allocate=allocator or _allocate
        self.bev_available=display.read(0x30)==BEV_CAPABILITY
        self.ui_available=display.read(0x300)==UI_CAPABILITY
        self.bev_depth_available=self.ui_available or display.read(0x30)==BEV2_CAPABILITY
        self.bev_color_available=self.bev_depth_available and display.read(0x34)==COLOR_CAPABILITY and mmio.read(0xc4)==RGB_CAPABILITY
        self.hud_available=display.read(0x38)==HUD_CAPABILITY
        self._hud_ped=0;self._ped_latch=AlertLatch(1)
        self.bev_fusion=None
        self._buffers=[];self._thread=None;self._stop=threading.Event();self._io=threading.RLock();self._condition=threading.Condition()
        self._latest=None;self._latest_mask=None;self._mask_requested=0;self._mask_completed=0
        self._timestamps=deque(maxlen=31);self._count=0;self._error=None;self._poisoned=False;self._closed=False
        self._instances.add(self)

    @classmethod
    def stop_existing(cls):
        for instance in list(cls._instances):instance.close()

    @property
    def error(self):return self._error

    @property
    def fps(self):
        with self._condition:
            return (len(self._timestamps)-1)/(self._timestamps[-1]-self._timestamps[0]) if len(self._timestamps)>1 else 0.0

    def _wait(self,predicate,message,timeout=None):
        deadline=time.monotonic()+(self.timeout if timeout is None else timeout)
        while not predicate():
            if time.monotonic()>=deadline:raise TimeoutError(message)
            time.sleep(.001)

    def _display_idle(self):
        status=self.display.read(4)
        if status&0x1c:raise RuntimeError('LDS-Net display publication failed: status=0x%08x'%status)
        return status&3==0

    def _configuration_idle(self):
        status=self.mmio.read(4)
        return bool(status&4) and status&0x13==0 and self.display.read(4)&0x23==0

    def start(self):
        if self._closed or self._poisoned:raise RuntimeError('LDS-Net backend is closed or has an undrained timeout')
        if self.bev_depth_available and self.bev_fusion is None:
            raise RuntimeError('BEV2 requires the matching SGM snapshot provider before enabling hardware')
        if self._thread is not None and self._thread.is_alive():return self
        self._wait(self._configuration_idle,'LDS-Net owners are still active; stop the prior backend while HDMI is running')
        self._ped_latch=AlertLatch(1);self._hud_ped=0
        if not self._buffers:
            try:
                for size in SIZES:
                    buffer=self._allocate(size);self._buffers.append(buffer)
                    if int(buffer.nbytes)<size:raise ValueError('CMA allocator returned an undersized buffer')
                    address=_address(buffer)
                    if address<=0 or address%64 or address+size>2**32:raise ValueError('CMA buffer requires an aligned32-bit DDR address')
                    buffer[:size]=0
                extents=sorted((_address(v),_address(v)+n) for v,n in zip(self._buffers,SIZES))
                if any(b[0]<a[1] for a,b in zip(extents,extents[1:])):raise ValueError('CMA allocations overlap')
                program=np.frombuffer((self.root/self.contract.get('compiled_program_file','sparse_program.bin')).read_bytes(),np.uint8)
                if program.nbytes!=self.contract['compiled_program_bytes'] or program.nbytes>SIZES[3]:
                    raise RuntimeError('Unexpected fixed-pair program size')
                self._buffers[3][:program.size]=program
                for buffer,size in zip(self._buffers,SIZES):_cache(buffer,size,'flush')
            except Exception:
                self._free();raise
        with self._io:
            for i,buffer in enumerate(self._buffers):self.mmio.write(16+i*4,_address(buffer))
            if self.mmio.read(0xc4)==RGB_CAPABILITY:self.mmio.write(0xc0,int(self.bev_color_available))
            self._stop.clear();self._error=None;self.display.write(4,4);self.mmio.write(4,17)
        self._wait(lambda:bool(self.mmio.read(4)&1) or self.mmio.read(8)!=0,'LDS-Net DDR layout validation timed out')
        if not self.mmio.read(4)&1:raise RuntimeError('LDS-Net rejected DDR layout: 0x%08x'%self.mmio.read(8))
        if self.bev_depth_available:
            self.bev_fusion.start()
        self._thread=threading.Thread(target=self._run,name='LDS-Net-results',daemon=True);self._thread.start()
        return self

    def configure_bev(self,snapshot,geometry):
        if self._thread is not None and self._thread.is_alive():raise RuntimeError('Configure BEV before starting LDS-Net')
        if not self.bev_depth_available:raise RuntimeError('BEV2 display hardware is required')
        ui=UIFramebuffer(self.display,self._allocate,_cache,self._io) if getattr(self,'ui_available',False) else None
        self.bev_fusion=BEVFusion(snapshot,geometry,self.display,self._io,ui=ui)

    def bev_diagnostics(self):
        return None if self.bev_fusion is None else self.bev_fusion.diagnostics()

    def _write_boxes(self,detections):
        for i,box in enumerate(detections[:16]):
            x1=max(80,min(559,math.floor(box['x1'])));y1=max(0,min(479,math.floor(box['y1'])))
            x2=max(80,min(559,math.ceil(box['x2'])));y2=max(0,min(479,math.ceil(box['y2'])))
            self.display.write(0x100+i*8,x1|(y1<<16));self.display.write(0x104+i*8,x2|(y2<<16))
        self.display.write(16,min(16,len(detections)))
        if self.bev_available:
            for i,value in enumerate(descriptors(detections)):
                self.display.write(BEV_BASE+i*4,value)

    def _process_result(self):
        began=time.monotonic()
        with self._io:
            if self._stop.is_set():return
            self.mmio.write(4,5)
            frame=int(self.mmio.read(0x60));slot=int(self.mmio.read(0x64));packet=int(self.mmio.read(0x68));mask=int(self.mmio.read(0x6c))
            if slot not in (0,1) or packet!=_address(self._buffers[4+slot]) or mask!=_address(self._buffers[6+slot]):
                raise RuntimeError('LDS-Net result metadata does not identify an owned buffer')
        packet_buffer=self._buffers[4+slot]
        _cache(packet_buffer,26200 if self.hud_available else 7000,'invalidate')
        road_logits=np.array(packet_buffer[7000:26200],dtype=np.int8,copy=True).reshape(3,80,80) if self.hud_available else None
        detections=self.decoder(np.array(packet_buffer[:7000],dtype=np.int8,copy=True),self.threshold)
        if self.bev_color_available:
            with self._io:
                rgb_address=int(self.mmio.read(0xc8));rgb_frame=int(self.mmio.read(0xcc))
            matches=[v for v in self._buffers[:2] if _address(v)==rgb_address]
            if rgb_frame!=frame or len(matches)!=1 or not self.mmio.read(4)&16:
                raise RuntimeError('RGB color source is not owned by this detection frame')
            rgb_buffer=matches[0];_cache(rgb_buffer,307200,'invalidate')
            detections=attach_colors(detections,rgb_buffer[:307200].view(np.int8).reshape(320,320,3),frame)
            # Stop/reset cannot silently turn samples from a recycled input
            # into a valid color. Stop joins this worker before freeing CMA.
            with self._io:
                if (self.mmio.read(0xc8)!=rgb_address or self.mmio.read(0xcc)!=frame or
                        self.mmio.read(0x60)!=frame or not self.mmio.read(4)&16):
                    raise RuntimeError('RGB frame ownership changed during color sampling')
        if self.hud_available:
            self._hud_ped=int(self._ped_latch.update(0,any(b['category']==0 for b in detections),time.monotonic()))
            if getattr(self,'ui_available',False):
                self.bev_fusion.ui.set_pedestrian(self._hud_ped)
            else:
                with self._io:self.display.write(0x260,self._hud_ped)
        decoded=time.monotonic()
        with self._condition:mask_request=self._mask_requested
        mask_copy=None
        if mask_request>self._mask_completed:
            _cache(self._buffers[6+slot],57600,'invalidate')
            mask_copy=unpack_mask(np.array(self._buffers[6+slot][:57600],copy=True).tobytes())
        self._wait(self._display_idle,'LDS-Net prior display publication did not complete')
        with self._io:
            if self._stop.is_set():
                if self.mmio.read(4)&16:self.mmio.write(4,8)
                return
            self._write_boxes(detections)
            self.display.write(20,frame);self.display.write(24,mask);self.display.write(28,slot);self.display.write(4,1)
        self._wait(self._display_idle,'LDS-Net box/mask publication did not reach HDMI SOF')
        if self.display.read(8)!=frame or self.display.read(12)!=slot:
            raise RuntimeError('LDS-Net display acknowledged a different frame')
        completed=time.monotonic()
        if self.bev_fusion is not None:
            if self.hud_available:self.bev_fusion.submit(frame,detections,road_logits)
            else:self.bev_fusion.submit(frame,detections)
        network_frame=int(self.mmio.read(0xbc))
        snapshot=dict(frame_id=frame,detections=[dict(v) for v in detections],mask=mask_copy,roi=CROP,
            bev_mode='bearing_x_metric_z' if self.bev_depth_available else 'image_relative_schematic' if self.bev_available else None,
            bev_color_available=self.bev_color_available,
            input_size=(320,320),completed_at=completed,postprocess_ms=(decoded-began)*1000,
            publish_ms=(completed-decoded)*1000,network_cycles=int(self.mmio.read(0x70)) if network_frame==frame else None)
        with self._condition:
            self._latest=snapshot;self._timestamps.append(completed);self._count+=1
            if mask_copy is not None:self._latest_mask=snapshot;self._mask_completed=mask_request
            self._condition.notify_all()

    def _run(self):
        try:
            while not self._stop.is_set():
                status=self.mmio.read(4)
                if status&64:raise RuntimeError('LDS-Net PL buffer ownership protocol failed')
                if status&8:self._process_result()
                else:self._stop.wait(.001)
        except Exception as error:
            if not self._stop.is_set():self._error=error
            self._stop.set()
            with self._io:
                self.mmio.write(4,2);self.display.write(4,2)
            with self._condition:self._condition.notify_all()

    def detections(self):
        with self._condition:
            if self._error is not None:raise RuntimeError('LDS-Net worker failed') from self._error
            return [] if self._latest is None else [dict(v) for v in self._latest['detections']]

    def perception(self,*,include_mask=False,timeout=5.0):
        with self._condition:
            request=0
            if include_mask:self._mask_requested+=1;request=self._mask_requested
            deadline=time.monotonic()+timeout
            while (self._mask_completed<request if include_mask else self._latest is None):
                if self._error is not None:raise RuntimeError('LDS-Net worker failed') from self._error
                if self._stop.is_set():raise RuntimeError('LDS-Net is stopped')
                remaining=deadline-time.monotonic()
                if remaining<=0:raise TimeoutError('No complete LDS-Net result within timeout')
                self._condition.wait(remaining)
            result=self._latest_mask if include_mask else self._latest
            return dict(result,detections=[dict(v) for v in result['detections']],mask=result['mask'].copy() if include_mask else None)

    def road_mask(self,*,full_frame=True,timeout=5.0):
        mask=self.perception(include_mask=True,timeout=timeout)['mask']
        return mask if full_frame else mask[:,80:560].copy()

    def diagnostics(self):
        with self._io:
            result=dict(abi=self.mmio.read(0),capabilities=self.mmio.read(12),clock_hz=self.mmio.read(0x44),
                captured=self.mmio.read(0x48),inferred=self.mmio.read(0x4c),restored=self.mmio.read(0x50),failed_jobs=self.mmio.read(0x54),
                input_states=self.mmio.read(0x58),output_states=self.mmio.read(0x5c),raw_drops=self.mmio.read(0xb0),crop_drops=self.mmio.read(0xb4),
                mask_cycles=self.mmio.read(0xb8),display_status=self.display.read(4),display_frame=self.display.read(8),
                cache_failures=self.display.read(32),read_errors=self.display.read(36),mask_misses=self.display.read(40))
        with self._condition:
            result.update(fps=self.fps,results=self._count,frame_id=None if self._latest is None else self._latest['frame_id'],
                result_age_ms=None if self._latest is None else (time.monotonic()-self._latest['completed_at'])*1000,
                error=None if self._error is None else str(self._error),poisoned=self._poisoned)
        return result

    def profiles(self):
        with self._io:
            for attempt in range(3):
                frame=self.mmio.read(0xbc);nodes=[]
                for index in range(50):
                    self.mmio.write(0x30,index)
                    nodes.append(dict(node=index,cycles=self.mmio.read(0x34),source_bytes=self.mmio.read(0x38),program_bytes=self.mmio.read(0x3c),destination_bytes=self.mmio.read(0x40)))
                if self.mmio.read(0xbc)==frame:return dict(frame_id=frame,nodes=nodes)
        raise RuntimeError('LDS-Net profile changed repeatedly during read')

    def stop(self,timeout=None):
        timeout=self.timeout if timeout is None else timeout
        self._stop.set()
        with self._io:self.mmio.write(4,2);self.display.write(4,2)
        try:
            if self.bev_fusion is not None:self.bev_fusion.stop(timeout)
            if self._thread is not None:
                self._thread.join(timeout)
                if self._thread.is_alive():raise TimeoutError('LDS-Net result worker did not stop')
            with self._io:
                if self.mmio.read(4)&16:self.mmio.write(4,8)
            self._wait(self._configuration_idle,'LDS-Net DMA/display did not drain; CMA buffers retained',timeout)
        except Exception:
            self._poisoned=True
            if not any(v is self for v in self._retained):self._retained.append(self)
            raise
        finally:
            with self._condition:self._condition.notify_all()
        self._thread=None

    def _free(self):
        for buffer in self._buffers:
            if hasattr(buffer,'freebuffer'):buffer.freebuffer()
            elif hasattr(buffer,'close'):buffer.close()
        self._buffers=[]

    def close(self):
        if self._closed:return
        self.stop()
        self._free();self._closed=True
        self._retained[:]=[v for v in self._retained if v is not self]

    def __enter__(self):return self.start()
    def __exit__(self,*_):self.close()
