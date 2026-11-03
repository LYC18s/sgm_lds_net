"""Raw-left center rectification, robust center disparity and metric Z."""
import json
import math
from pathlib import Path
import numpy as np


class BEVGeometry:
    def __init__(self,calibration,window=5,depth_scale=12.):
        if window not in (3,5) or not math.isfinite(depth_scale) or depth_scale<=0:
            raise ValueError('Invalid depth scale/window')
        self.k=np.array(calibration['camera_matrix_left'],float)
        self.d=np.array(calibration['distortion_left'],float).reshape(-1)
        self.r=np.array(calibration['R_left'],float)
        self.p=np.array(calibration['P_left'],float)[:,:3]
        if self.d.size!=5:raise ValueError('Expected the fixed rig five-coefficient distortion model')
        self.fb=abs(float(calibration['P_right'][0][3]))/1000.
        if not math.isfinite(self.fb) or self.fb<=0:raise ValueError('Invalid stereo focal-baseline product')
        self.window,self.depth_scale=window,float(depth_scale)

    @classmethod
    def from_directory(cls,directory,**kwargs):
        root=Path(directory)
        c=json.loads((root/'fixed_rig_intrinsics.json').read_text(encoding='utf-8'))
        rig=json.loads((root/'fixed_rig.json').read_text(encoding='utf-8'))
        if c['source_zip_sha256']!=rig['source_zip_sha256'] or c['logical_to_physical_camera']!=[1,0]:
            raise RuntimeError('BEV calibration does not match installed stereo rig')
        return cls(c,**kwargs)

    def rectify(self,u,v):
        if not math.isfinite(u) or not math.isfinite(v):return None
        ray=np.linalg.solve(self.k,np.array([u,v,1.]))
        xd,yd=ray[:2]/ray[2];x,y=xd,yd
        k1,k2,p1,p2,k3=self.d
        for _ in range(30):
            rr=x*x+y*y;radial=1+k1*rr+k2*rr*rr+k3*rr**3
            if not math.isfinite(radial) or abs(radial)<1e-9:return None
            dx=2*p1*x*y+p2*(rr+2*x*x);dy=p1*(rr+2*y*y)+2*p2*x*y
            x,y=(xd-dx)/radial,(yd-dy)/radial
        rr=x*x+y*y;radial=1+k1*rr+k2*rr*rr+k3*rr**3
        if abs(x*radial+2*p1*x*y+p2*(rr+2*x*x)-xd)>1e-6:return None
        if abs(y*radial+p1*(rr+2*y*y)+2*p2*x*y-yd)>1e-6:return None
        point=self.p.dot(self.r.dot([x,y,1.]))
        if point[2]<=0:return None
        q=point[:2]/point[2]
        return tuple(float(a) for a in q) if np.all(np.isfinite(q)) else None

    def distance(self,patch):
        if patch is None:return dict(valid=False,reason='outside_rectified_frame')
        data=np.asarray(patch,dtype=float).reshape(-1)
        values=data[np.isfinite(data)&(data>0)&(data<=63)]
        minimum=9 if self.window==5 else 5
        if len(values)<minimum:return dict(valid=False,reason='too_few_valid_samples',valid_samples=len(values))
        median=float(np.median(values));kept=values[np.abs(values-median)<=2.]
        if len(kept)<minimum or len(kept)<.6*len(values):
            return dict(valid=False,reason='inconsistent_disparity',valid_samples=len(values),kept_samples=len(kept))
        disparity=float(np.mean(kept));z=self.fb/disparity
        # Measurement validity is independent of display depth. Never discard a
        # stable positive disparity merely because the target is farther away.
        result=dict(valid=math.isfinite(z) and z>0,disparity=disparity,distance_m=z,
            valid_samples=len(values),kept_samples=len(kept),spread=float(np.std(kept)))
        if not result['valid']:result['reason']='invalid_metric_depth'
        return result

    @staticmethod
    def sprite_size(kind,grade):
        return (((67,96),(57,80),(45,64),(34,48)) if kind==1 else
            ((24,48),(20,40),(16,32),(12,24)))[grade]

    def project(self,detections,snapshot):
        descriptors=[];diagnostics=[]
        for box,center,patch in zip(detections,snapshot['centers'],snapshot['patches']):
            measured=self.distance(patch)
            measured.update(category=int(box['category']),raw_center=[(box['x1']+box['x2'])/2,(box['y1']+box['y2'])/2],
                rectified_center=center)
            if measured['valid']:
                kind=2 if box['category']==0 else 1
                z=measured['distance_m']
                grade=sum(z>=limit for limit in (8.,20.,45.))
                width,height=self.sprite_size(kind,grade)
                # Both panels are 640 pixels wide: preserve the raw-left
                # horizontal fraction exactly, independent of depth or lanes.
                cx=measured['raw_center'][0]
                # Continuous, monotonic perspective compression; y tends to 35
                # for distant objects without a far clipping threshold.
                cy=285-250/(1+self.depth_scale/z)
                left=int(round(cx-width/2));top=int(round(cy-height/2))
                if not (-width<left<640 and 0<=top<=480-height):
                    measured.update(valid=False,reason='outside_display')
                else:
                    color=box.get('vehicle_color',{}) if kind==1 else {}
                    color_class=int(color.get('color_class',0))
                    if not 0<=color_class<=9:color_class=0
                    # X 640..1023 encodes negative origins modulo 1024. This
                    # allows a sprite at x=0 to clip instead of moving inward.
                    word=(left&1023)|(top<<10)|(kind<<19)|(grade<<21)|(color_class<<23)
                    descriptors.append((z,len(diagnostics),word))
                    measured.update(display_center=[cx,cy],size_grade=grade,sprite_size=[width,height])
                    if kind==1:measured['vehicle_color']=dict(color,color_class=color_class)
            diagnostics.append(measured)
        # Near first: tie-breaking at equal raster depth also respects metric Z.
        descriptors.sort(key=lambda item:item[0])
        for slot,(_,index,_) in enumerate(descriptors):diagnostics[index]['descriptor_slot']=slot
        return ([item[2] for item in descriptors]+[0]*16)[:16],diagnostics


def select_targets(detections):
    result=[];candidates=[]
    for value in detections:
        try:
            b={k:float(value[k]) for k in ('x1','y1','x2','y2','score')};b['category']=int(value['category'])
        except (ValueError,TypeError,KeyError,OverflowError):continue
        if b['category'] not in (0,2,4,5) or not all(math.isfinite(b[k]) for k in ('x1','y1','x2','y2','score')):continue
        if b['x2']<=b['x1'] or b['y2']<=b['y1'] or b['x2']<=0 or b['y2']<=0 or b['x1']>=640 or b['y1']>=480:continue
        b['x1']=max(0.,b['x1']);b['y1']=max(0.,b['y1'])
        b['x2']=min(640.,b['x2']);b['y2']=min(480.,b['y2'])
        if isinstance(value.get('vehicle_color'),dict):b['vehicle_color']=dict(value['vehicle_color'])
        candidates.append(b)
    for b in sorted(candidates,key=lambda b:-b['score']):
        duplicate=False
        for a in result:
            intersection=max(0,min(a['x2'],b['x2'])-max(a['x1'],b['x1']))*max(0,min(a['y2'],b['y2'])-max(a['y1'],b['y1']))
            union=(a['x2']-a['x1'])*(a['y2']-a['y1'])+(b['x2']-b['x1'])*(b['y2']-b['y1'])-intersection
            if (a['category']==0)==(b['category']==0) and intersection/union>=.6:duplicate=True;break
        if not duplicate:result.append(b)
        if len(result)==16:break
    return result
