"""Standalone LDS-Net integer inference: no Ultralytics or PyTorch pickle loading."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import torch
from lds_net.data import letterbox
from lds_net.integer_reference import IntegerReference
from lds_net.sparse_weights import SparseIntegerReference
from lds_net.ps_postprocess import decode_detection,restore_road_int8


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[1] / 'model')
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dense',action='store_true')
    p.add_argument('--confidence',type=float,default=.25)
    args=p.parse_args();torch.set_num_threads(4)
    image=cv2.imread(str(args.image))
    if image is None:raise OSError(args.image)
    square,_,geometry=letterbox(image)
    rgb=np.ascontiguousarray(square[:,:,::-1].transpose(2,0,1))
    q=(rgb.astype(np.int16)-128).astype(np.int8)[None]
    ref=IntegerReference(args.bundle/'dense_integer') if args.dense else SparseIntegerReference(args.bundle/'sparse_integer')
    codes=ref(q);raw=ref.dequantize(codes)
    det=decode_detection(raw)[0];det=det[det[:,4]>=args.confidence]
    ratio,top,left,_,_=geometry
    det[:,[0,2]]=(det[:,[0,2]]-left)/ratio;det[:,[1,3]]=(det[:,[1,3]]-top)/ratio
    det[:,[0,2]]=det[:,[0,2]].clamp(0,image.shape[1]);det[:,[1,3]]=det[:,[1,3]].clamp(0,image.shape[0])
    labels=restore_road_int8(codes[-1],image.shape[:2],geometry)[0].byte().numpy()
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(args.output/'road_labels.png'),labels)
    (args.output/'detections.json').write_text(json.dumps(det.tolist(),indent=2),encoding='utf-8')
    np.savez(args.output/'raw_int8.npz',**{f'output_{i}':v.numpy() for i,v in enumerate(codes)})
    print(json.dumps({'passed':True,'detections':len(det),'road_shape':list(labels.shape),
        'road_categories':np.unique(labels).tolist(),'nms':False,'bundle':str(args.bundle.resolve())}))


if __name__=='__main__':main()
