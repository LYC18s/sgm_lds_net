"""Cached integer sprite renderer; no camera, inference or disparity ownership."""
import math
from pathlib import Path
import numpy as np
from .bev_color import paint_palette

BEV_BYTES=640*480*3
TILE_BYTES=192*128*3
UI_BYTES=BEV_BYTES+5*TILE_BYTES
POSITIONS=((224,866),(864,866),(1504,866),(224,86),(1504,86))
BRIGHT=((255,208,0),(255,80,60),(255,64,64),(0,238,132),(0,176,255))
DARK=((42,34,0),(42,13,10),(42,10,10),(0,39,22),(0,29,42))
SIZES=(((67,96),(57,80),(45,64),(34,48)),((24,48),(20,40),(16,32),(12,24)))


class UIRenderer:
    def __init__(self,assets=None):
        path=Path(assets) if assets else Path(__file__).with_name('assets')/'ps_ui.npz'
        with np.load(str(path),allow_pickle=False) as data:
            self.background=data['background'].copy()
            self.indices=[data['indices'+str(i)].copy() for i in range(3)]
            self.palettes=[data['palette'+str(i)].copy() for i in range(3)]
            self.hud=data['hud'].copy()
        self.cache={}
        self.tiles=[[np.where(self.hud[k,:,:,None],np.array(colors[k],np.uint8),0).astype(np.uint8)
                     for k in range(5)] for colors in (DARK,BRIGHT)]
        self.ego=self._sprite(0,0,0,False,0)
        # Warm all geometry masks. Colors remain a tiny indexed palette lookup.
        for kind in (1,2):
            for grade in range(4):
                for tilt in (range(5) if kind==1 else (0,)):
                    for left in (False,True):self._sprite(kind,grade,tilt,left,0)

    def _sprite(self,kind,grade,tilt,left,paint):
        key=(kind,grade,tilt,left)
        if key not in self.cache:
            w,h=(67,90) if kind==0 else SIZES[kind-1][grade]
            margin=16 if kind==1 else 0
            yy,xx=np.indices((h+2*margin,w+2*margin),dtype=np.int32)
            xx=2*(xx-margin)-w+1;yy=2*(yy-margin)-h+1
            angle=math.radians((0,4,5,6,8)[tilt])
            c=int(math.cos(angle)*256+.5);s=int(math.sin(angle)*256+.5)*(1 if left else -1)
            sx=(c*xx+s*yy+w*256)//512;sy=(-s*xx+c*yy+h*256)//512
            valid=(sx>=0)&(sx<w)&(sy>=0)&(sy<h)
            source=self.indices[kind];oh,ow=source.shape
            ix=np.zeros(valid.shape,np.uint8)
            ix[valid]=source[sy[valid]*oh//h,sx[valid]*ow//w]
            self.cache[key]=(ix,ix!=0,margin)
        ix,mask,margin=self.cache[key]
        palette=paint_palette(self.palettes[1],paint if paint<10 else 0) if kind==1 else self.palettes[kind]
        return np.asarray(palette,np.uint8)[ix],mask,margin

    @staticmethod
    def _paste(canvas,left,top,sprite):
        rgb,mask,margin=sprite;left-=margin;top-=margin
        h,w=mask.shape;x0=max(0,left);y0=max(0,top);x1=min(640,left+w);y1=min(480,top+h)
        if x0>=x1 or y0>=y1:return
        a=slice(y0-top,y1-top);b=slice(x0-left,x1-left)
        np.copyto(canvas[y0:y1,x0:x1],rgb[a,b],where=mask[a,b,None])

    def render(self,buffer,words,flags=0):
        data=np.asarray(buffer,dtype=np.uint8).reshape(-1)
        if data.size<UI_BYTES:raise ValueError('UI allocation is too small')
        canvas=data[:BEV_BYTES].reshape(480,640,3);np.copyto(canvas,self.background)
        targets=[]
        for slot,word in enumerate(words[:16]):
            word=int(word);x=word&1023;x=x if x<640 else x-1024
            y=(word>>10)&511;kind=(word>>19)&3;grade=(word>>21)&3;paint=(word>>23)&15
            if kind not in (1,2):continue
            w,h=SIZES[kind-1][grade]
            if x>=640 or x+w<=0 or y>480-h:continue
            center2=2*x+w-1;tilt=min(4,abs(center2-640)//128) if kind==1 else 0
            targets.append((2*y+h,-slot,x,y,kind,grade,tilt,center2<640,paint))
        for _,__,x,y,kind,grade,tilt,left,paint in sorted(targets):
            self._paste(canvas,x,y,self._sprite(kind,grade,tilt,left,paint))
        self._paste(canvas,287,309,self.ego)
        for kind in range(5):
            start=BEV_BYTES+kind*TILE_BYTES
            data[start:start+TILE_BYTES]=self.tiles[bool(flags&(1<<kind))][kind].reshape(-1)
        return data[:UI_BYTES]
