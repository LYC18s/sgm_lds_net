"""Latest-job depth fusion, independent of CNN mask/box publication."""
import threading
import time
from collections import deque
import copy
from .bev_geometry import select_targets
from .sgm_snapshot import SnapshotCancelled
from .alerts import HUD_CAPABILITY,AlertPolicy,targets_for_alerts

BEV2_CAPABILITY=0x42455632


class BEVFusion:
    def __init__(self,snapshot,geometry,display,io_lock,ui=None):
        self.snapshot,self.geometry,self.display,self.io=snapshot,geometry,display,io_lock
        self.ui=ui
        self.stop_event=threading.Event();self.condition=threading.Condition();self.thread=None
        self.job=None;self.latest=None;self.error=None;self.dropped=0;self.count=0
        self.timestamps=deque(maxlen=31)
        self.alert_enabled=display.read(0x38)==HUD_CAPABILITY;self.alert_policy=AlertPolicy();self.alert_state=None

    def _wait_idle(self,timeout=.5,ignore_stop=False):
        deadline=time.monotonic()+timeout
        while self.display.read(0x24c)&1:
            if not ignore_stop and self.stop_event.is_set():raise SnapshotCancelled('BEV stopped')
            if time.monotonic()>deadline:raise TimeoutError('BEV publication did not reach SOF')
            time.sleep(.001)

    def clear(self,clear_ped=False):
        if self.ui is not None:
            self.ui.submit(0,0,[0]*16,0);return
        with self.io:
            if clear_ped and self.alert_enabled:self.display.write(0x260,0)
            self.display.write(0x240,2)
        self._wait_idle(ignore_stop=True)

    def publish(self,frame,sequence,values,alert_bits=0):
        if self.ui is not None:
            self.ui.submit(frame,sequence,values,alert_bits);return
        self._wait_idle()
        if self.stop_event.is_set():raise SnapshotCancelled('BEV stopped')
        with self.io:
            for index,value in enumerate(values):self.display.write(0x200+4*index,value)
            if self.alert_enabled:self.display.write(0x258,alert_bits)
            self.display.write(0x244,frame);self.display.write(0x248,sequence);self.display.write(0x240,1)
        self._wait_idle()
        if self.display.read(0x250)!=frame or self.display.read(0x254)!=sequence:
            raise RuntimeError('BEV acknowledged a different detection/depth pair')

    def start(self):
        if self.thread is not None and self.thread.is_alive():return
        self.stop_event.clear();self.error=None;self.alert_policy.reset();self.alert_state=None;self.clear(clear_ped=True)
        if self.ui is not None:
            try:self.ui.start();self.clear(clear_ped=True)
            except Exception as error:
                self.ui.error=str(error);self.ui.stopping=True
        self.thread=threading.Thread(target=self._run,name='BEV-depth',daemon=True);self.thread.start()

    def submit(self,frame,detections,road_logits=None):
        with self.condition:
            if self.stop_event.is_set():return
            if self.job is not None:self.dropped+=1
            self.job=(int(frame),copy.deepcopy(detections),None if road_logits is None else road_logits.copy(),time.monotonic())
            self.condition.notify_all()

    def _run(self):
        while not self.stop_event.is_set():
            with self.condition:
                while self.job is None and not self.stop_event.is_set():self.condition.wait(.1)
                if self.stop_event.is_set():return
                frame,detections,road_logits,queued=self.job;self.job=None
            try:
                boxes=select_targets(detections)
                if time.monotonic()-queued>.75:
                    self.dropped+=1;continue
                alert_boxes=targets_for_alerts(detections) if self.alert_enabled else []
                measurements=boxes+alert_boxes
                if measurements:
                    points=[self.geometry.rectify((b['x1']+b['x2'])/2,(b['y1']+b['y2'])/2) for b in measurements]
                    unique=[];indices=[]
                    for point in points:
                        if point not in unique:unique.append(point)
                        indices.append(unique.index(point))
                    snapshot=self.snapshot.sample_next(unique,radius=self.geometry.window//2,stop=self.stop_event)
                    expanded=dict(snapshot,centers=[snapshot['centers'][i] for i in indices],patches=[snapshot['patches'][i] for i in indices])
                    values,targets=self.geometry.project(boxes,dict(expanded,centers=expanded['centers'][:len(boxes)],patches=expanded['patches'][:len(boxes)]))
                    sequence=snapshot['sgm_sequence'];request=snapshot['request_id']
                    distances=[]
                    for patch in expanded['patches'][len(boxes):]:
                        measured=self.geometry.distance(patch)
                        distances.append(measured['distance_m'] if measured['valid'] else None)
                else:
                    values=[0]*16;targets=[];sequence=0;request=None;distances=[]
                if self.alert_enabled:self.alert_state=self.alert_policy.update(frame,detections,distances,road_logits)
                self.publish(frame,sequence,values,0 if self.alert_state is None else self.alert_state['bits'])
                self.count+=1;self.error=None
                with self.condition:self.timestamps.append(time.monotonic())
                self.latest=dict(detection_frame=frame,sgm_sequence=sequence,request_id=request,
                    alerts=copy.deepcopy(self.alert_state),targets=targets,elapsed_ms=(time.monotonic()-queued)*1000,updates=self.count,dropped=self.dropped)
            except SnapshotCancelled:
                if not self.stop_event.is_set():self.error='Unexpected cancellation'
            except Exception as error:
                self.error=str(error)
                self.alert_policy.reset();self.alert_state=None
                # No old or image-relative fallback distances. Clear stale targets.
                try:self.clear()
                except Exception as clearing_error:self.error+='; clear: '+str(clearing_error)

    def stop(self,timeout=2.):
        self.stop_event.set()
        with self.condition:self.job=None;self.condition.notify_all()
        if self.thread is not None:
            self.thread.join(timeout)
            if self.thread.is_alive():raise TimeoutError('BEV worker still owns a frame lease')
        if self.ui is not None:self.ui.stop(timeout)
        else:self.clear(clear_ped=True)
        self.thread=None

    def diagnostics(self):
        with self.condition:times=tuple(self.timestamps)
        fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
        return dict(mode='bearing_x_metric_z',ui=None if self.ui is None else self.ui.diagnostics(),latest=copy.deepcopy(self.latest),error=self.error,dropped=self.dropped,
            updates=self.count,fps=fps,window=self.geometry.window,
            depth_mapping='y=285-250*z/(z+depth_scale)',depth_scale_m=self.geometry.depth_scale,
            far_clip_m=None,size_thresholds_m=[8.,20.,45.])
