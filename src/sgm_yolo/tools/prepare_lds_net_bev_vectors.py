"""Independent integer pixel reference for the hardware BEV stream test."""
import json
from pathlib import Path
import numpy as np
import sys
import math

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from dual_ov5640.bev_color import paint_palette
SCALED_TARGETS=((240,180,1,1,4),(380,180,1,2,5),(300,45,1,3,1),
    (270,220,2,1),(400,110,2,2),(400,60,2,3),(610,400,1,3,3),(630,420,2,3),
    (250,198,1,1,1),(-12,120,1,1,6),(620,120,1,1,7),(490,120,1,1,8))
TARGETS=(((299,118,1),(369,252,2),*SCALED_TARGETS),((283,102,1,0,9),(363,236,2),*SCALED_TARGETS))


def reference(targets):
    atlas=json.loads((ROOT/'hardware/assets/bev/rom.json').read_text())
    y,x=np.indices((480,640))
    edge=28+y//8+y//32
    # Independent floating model sampled at Q4 palette nodes, then Q8 interpolation.
    # This verifies the generated RTL table, interpolation, quantization and phase.
    knots=np.arange(33)*16
    light=(1-np.exp(-5*(np.minimum(knots,480)/480.)**2))/(1-np.exp(-5))
    def ramp(a,b):return np.array(a)[None,:]+(np.array(b)-a)[None,:]*light[:,None]
    road_knots=ramp((14,21,33),(170,192,220))
    guide_knots=ramp((14,22,35),(170,210,253))
    alpha=np.clip((knots-32)/256.,0,1);alpha=alpha**2*(3-2*alpha)
    path_knots=guide_knots*(1-alpha[:,None])+np.array([52,114,253])*alpha[:,None]
    dash_knots=ramp((16,18,24),(252,254,255))
    nodes=np.floor(np.stack((road_knots,guide_knots,path_knots,dash_knots),axis=1)*16+.5).astype(int)
    rows=np.arange(480);index=rows//16
    palette=nodes[index]*16+(nodes[index+1]-nodes[index])*(rows%16)[:,None,None]
    bayer=np.array([[0,8,2,10],[12,4,14,6],[3,11,1,9],[15,7,13,5]])
    threshold=bayer[y%4,x%4]*16+8
    def color(bank):
        q=palette[:,None,bank,:]
        return np.minimum(255,q//256+(q%256>threshold[:,:,None]))
    road=color(0);guidance_rgb=color(1);path_rgb=color(2);dash_rgb=color(3)
    # Curve only the backdrop's sample depth. All foreground colors use original y.
    background_y_q4=np.maximum(y*16-(5*(x-320)**2)//256,0)
    bg_index=background_y_q4//256;bg_fraction=background_y_q4%256
    bg_q=nodes[bg_index,0,:]*16+(nodes[bg_index+1,0,:]-nodes[bg_index,0,:])*bg_fraction[:,:,None]//16
    background=np.minimum(255,bg_q//256+(bg_q%256>threshold[:,:,None]))
    image=background.astype(np.uint8)
    guidance_half=edge-edge//4
    guidance=(x>=320-guidance_half)&(x<=320+guidance_half)
    path_half=edge//4+edge//16
    path=(y>=32)&(y<336)&(x>=320-path_half)&(x<=320+path_half)
    image[guidance]=guidance_rgb[guidance]
    image[path]=path_rgb[path]
    top_spread=np.maximum(8-y//32,0)
    dash_edge=44+y//16+y//32+top_spread
    outer=dash_edge+83+y//8+y//32+2*(y//64)+top_spread
    boundary=((x>=320-dash_edge)&(x<323-dash_edge))|((x<=320+dash_edge)&(x+3>320+dash_edge))
    outer_boundary=((x>=320-outer)&(x<323-outer))|((x<=320+outer)&(x+3>320+outer))
    dash=np.where(y<128,(y//8)%2==0,np.where(y<288,(y//16)%2==0,y%64<36))
    image[outer_boundary&dash]=(road+[8,10,12])[outer_boundary&dash]
    image[boundary&dash]=dash_rgb[boundary&dash]
    image[0]=image[-1]=[51,68,86]
    image[:,0]=image[:,-1]=[51,68,86]
    # Independent painter's algorithm: far sprites first, near sprites last;
    # transparent pixels leave the previous layer intact.
    def sprite(left,top,bank,width,height,paint=0):
        original_width,original_height=atlas['sprites'][bank]['size']
        base=atlas['sprites'][bank]['offset']
        indices=np.array(atlas['pixels'][base:base+original_width*original_height]).reshape(original_height,original_width)
        palette=paint_palette(atlas['palettes'][1],paint if paint<10 else 0) if bank==1 else atlas['palettes'][bank]
        # Independent inverse rotation, pixel centers fixed; nearest sampling.
        center=left+(width-1)/2
        angle=(0,4,5,6,8)[min(4,int(abs(center-320)//64))] if bank==1 else 0
        angle=math.radians(angle)*(1 if center<320 else -1)
        cosine=int(math.cos(angle)*256+.5)
        sine=int(math.sin(abs(angle))*256+.5)*(1 if angle>=0 else -1)
        margin=16 if bank==1 else 0
        for row in range(-margin,height+margin):
            for col in range(-margin,width+margin):
                xx=2*col-width+1;yy=2*row-height+1
                sample_x=(cosine*xx+sine*yy+width*256)//512
                sample_y=(-sine*xx+cosine*yy+height*256)//512
                if not (0<=sample_x<width and 0<=sample_y<height):continue
                sy=sample_y*original_height//height
                sx=sample_x*original_width//width
                index=int(indices[sy,sx])
                if index and 0<=left+col<640 and 0<=top+row<480:
                    image[top+row,left+col]=palette[index]
    def center_y(item):
        index,target=item;kind=target[2];grade=target[3] if len(target)>3 else 0
        h=((96,80,64,48) if kind==1 else (48,40,32,24))[grade]
        return target[1]*2+h,-index
    for _,target in sorted(enumerate(targets),key=center_y):
        left,top,kind=target[:3];grade=target[3] if len(target)>3 else 0
        width,height=(((67,96),(57,80),(45,64),(34,48)) if kind==1 else
            ((24,48),(20,40),(16,32),(12,24)))[grade]
        paint=target[4] if len(target)>4 else 0
        sprite(left,top,kind,width,height,paint)
    sprite(287,309,0,67,90)
    return image


def main():
    out=ROOT/'build/lds_net/bev_vectors';out.mkdir(parents=True,exist_ok=True)
    atlas=json.loads((ROOT/'hardware/assets/bev/rom.json').read_text())
    (out/'rom.mem').write_text(''.join(f'{v:x}\n' for v in atlas['pixels']))
    for index,targets in enumerate(((),*TARGETS)):
        image=reference(targets)
        packed=(image[:,:,0].astype(np.uint32)<<16)|(image[:,:,1].astype(np.uint32)<<8)|image[:,:,2]
        (out/f'frame{index}.mem').write_text(''.join(f'{v:06x}\n' for v in packed.flat))
    print('PASS: three independent BEV golden pixel frames prepared')


if __name__=='__main__':main()
