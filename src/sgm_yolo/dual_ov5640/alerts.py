"""Frame-owned HUD decisions. No inference, pixel rendering or control of the vehicle."""
import time
import numpy as np

HUD_CAPABILITY=0x48554431

class AlertConfig:
    # PYNQ 2.5 uses Python 3.6, which has no standard-library dataclasses.
    def __init__(self,center_min=192,center_max=448,slow_enter=20,slow_exit=22,
                 obstacle_enter=10,obstacle_exit=12,min_area=60,min_span=12,
                 min_width=5,timeout=3,min_hold=1.0):
        self.center_min,self.center_max=center_min,center_max
        self.slow_enter,self.slow_exit=slow_enter,slow_exit
        self.obstacle_enter,self.obstacle_exit=obstacle_enter,obstacle_exit
        self.min_area,self.min_span,self.min_width=min_area,min_span,min_width
        self.timeout=timeout
        self.min_hold=min_hold


class AlertLatch:
    """Immediate activation; count two misses only after minimum on-time."""
    def __init__(self,count=5,min_hold=1.0):
        self.min_hold=float(min_hold);self.bits=0
        self.since=[None]*count;self.misses=[0]*count

    def update(self,index,on,now):
        bit=1<<index
        if on:
            if not self.bits&bit:self.since[index]=now
            self.bits|=bit;self.misses[index]=0
        elif self.bits&bit:
            if now-self.since[index]<self.min_hold:self.misses[index]=0
            else:
                self.misses[index]+=1
                if self.misses[index]>=2:
                    self.bits&=~bit;self.since[index]=None;self.misses[index]=0
        return bool(self.bits&bit)


def targets_for_alerts(detections):
    result=[]
    for box in detections:
        try:
            b={k:float(box[k]) for k in ('x1','y1','x2','y2','score')};b['category']=int(box['category'])
            if b['category']==6 or not 0<=b['category']<10:continue
            if not all(np.isfinite(b[k]) for k in ('x1','y1','x2','y2','score')):continue
            if b['x2']<=b['x1'] or b['y2']<=b['y1']:continue
            if not (0< b['x2'] and b['x1']<640 and 0<b['y2'] and b['y1']<480):continue
            result.append(b)
        except (ValueError,TypeError,KeyError,OverflowError):continue
    return result


def road_regions(logits,config):
    if logits is None:return [None,None]
    a=np.asarray(logits)
    if a.shape!=(3,80,80) or a.dtype!=np.int8:raise ValueError('Expected signed 3x80x80 logits')
    # Extract complete alternative-road components first. Direct road must not
    # bridge separate alternatives into one region across the current road.
    labels=a.argmax(0);road=labels>0
    # Remove isolated pixels without joining holes or disconnected components.
    p=np.pad(road.astype(np.uint8),1,mode='constant',constant_values=0)
    neighbors=p[:-2,1:-1]+p[2:,1:-1]+p[1:-1,:-2]+p[1:-1,2:]
    road &= (neighbors>=2)&(labels==2)
    results=[None,None]
    mask=road.copy();mask[:36]=False;mask[76:]=False
    seen=np.zeros((80,80),bool)
    for y,x in zip(*np.nonzero(mask)):
        if seen[y,x]:continue
        stack=[(int(y),int(x))];seen[y,x]=True;points=[]
        while stack:
            yy,xx=stack.pop();points.append((yy,xx))
            for ny,nx in ((yy-1,xx),(yy+1,xx),(yy,xx-1),(yy,xx+1)):
                if 0<=ny<80 and 0<=nx<80 and mask[ny,nx] and not seen[ny,nx]:
                    seen[ny,nx]=True;stack.append((ny,nx))
        rows={}
        for yy,xx in points:rows.setdefault(yy,[]).append(xx)
        ys=[q[0] for q in points];xx=[q[1] for q in points]
        wide=sum(max(v)-min(v)+1>=config.min_width for v in rows.values())
        if len(points)<config.min_area or max(ys)-min(ys)+1<config.min_span or wide<config.min_span or max(ys)<60:continue
        center_x=80+6*(sum(xx)/len(xx)+.5)-.5
        # Use the camera-image center, independent of the warning thresholds.
        # A component centered within one semantic cell has no clear direction.
        if abs(center_x-319.5)<=3:continue
        side=0 if center_x<319.5 else 1
        item=dict(area=len(points),alternative=len(points),xmin=80+6*min(xx),
                  xmax=80+6*(max(xx)+1),center_x=center_x)
        if results[side] is None or item['area']>results[side]['area']:results[side]=item
    return results


class AlertPolicy:
    def __init__(self,config=None):
        self.config=config or AlertConfig();self.reset()
    def reset(self):
        self.bits=0;self.latch=AlertLatch(5,self.config.min_hold);self.updated=None;self.last_frame=None
    def update(self,frame,detections,distances,logits,now=None):
        now=time.monotonic() if now is None else now;c=self.config
        if self.last_frame is not None and ((int(frame)-self.last_frame)&0xffffffff)>=0x80000000:return dict(bits=self.bits,ignored=True)
        if self.last_frame==frame:return dict(bits=self.bits,ignored=True)
        if self.updated is not None and now-self.updated>c.timeout:self.reset()
        boxes=targets_for_alerts(detections)
        if len(boxes)!=len(distances):raise ValueError('Distance list does not match alert targets')
        regions=road_regions(logits,c)
        central=[(b,z) for b,z in zip(boxes,distances) if c.center_min<=(b['x1']+b['x2'])/2<=c.center_max]
        known=[z for _,z in central if z is not None and np.isfinite(z) and z>0]
        unknown=any(z is None or not np.isfinite(z) or z<=0 for _,z in central)
        raw=[any(b['category']==0 for b in boxes),any(z<=(c.slow_exit if self.bits&2 else c.slow_enter) for z in known),any(z<=(c.obstacle_exit if self.bits&4 else c.obstacle_enter) for z in known)]
        blocked=[]
        for i,region in enumerate(regions):
            occupied=region is not None and any(region['xmin']<=(b['x1']+b['x2'])/2<region['xmax'] for b in boxes)
            blocked.append(occupied)
            eligible=bool((raw[1] or raw[2]) and not unknown and region is not None and not occupied)
            raw.append(eligible)
        for i,on in enumerate(raw):self.latch.update(i,on,now)
        self.bits=self.latch.bits
        self.updated=now;self.last_frame=int(frame)
        return dict(bits=self.bits,regions=regions,blocked=blocked,central_unknown=unknown,distances=distances)
