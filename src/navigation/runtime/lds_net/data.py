"""Standalone preprocessing and JSON writing; no dataset or training dependency."""
import json
from pathlib import Path
import cv2
import numpy as np


def write_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


def letterbox(image, mask=None, size=320):
    h,w=image.shape[:2];ratio=min(size/h,size/w)
    nh,nw=round(h*ratio),round(w*ratio)
    top,left=(size-nh)//2,(size-nw)//2
    result=np.full((size,size,3),114,np.uint8)
    result[top:top+nh,left:left+nw]=cv2.resize(image,(nw,nh),interpolation=cv2.INTER_LINEAR)
    output=None
    if mask is not None:
        output=np.full((size,size),255,np.uint8)
        output[top:top+nh,left:left+nw]=cv2.resize(mask,(nw,nh),interpolation=cv2.INTER_NEAREST)
    return result,output,(ratio,top,left,nh,nw)
