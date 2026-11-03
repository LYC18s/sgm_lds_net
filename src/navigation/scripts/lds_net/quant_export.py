"""Fixed INT8 I/O QCDQ check graph with explicit integer requantization."""
from pathlib import Path
import json
import numpy as np
import torch
import onnx
from onnx import helper as H,numpy_helper as N,TensorProto as T
from .deploy_graph import OUTPUT_NAMES
from .integer_reference import IntegerReference
from .data import write_json


def export_onnx(directory):
    directory=Path(directory)
    ref=IntegerReference(directory);spec=ref.spec
    initial=[];nodes=[];serial=[0]
    def const(value,name=None):
        if name is None:
            serial[0]+=1;name=f'constant_{serial[0]}'
        initial.append(N.from_array(np.asarray(value),name));return name
    def op(kind,*args,**attrs):
        serial[0]+=1;name=f'{kind}_{serial[0]}'
        nodes.append(H.make_node(kind,list(args),[name],name=name,**attrs));return name
    def cast(a,t):return op('Cast',a,to=t)
    zero64=const(np.array(0,np.int64));two64=const(np.array(2,np.int64))
    def rne(product,shifts):
        divisor=const(np.left_shift(np.ones_like(shifts,dtype=np.int64),shifts))
        half=const(np.left_shift(np.ones_like(shifts,dtype=np.int64),np.maximum(shifts-1,0)))
        neg=op('Less',product,zero64);absolute=op('Abs',product)
        whole=op('Div',absolute,divisor)
        remain=op('Sub',absolute,op('Mul',whole,divisor))
        odd=op('Equal',op('Mod',whole,two64),const(np.array(1,np.int64)))
        increment=op('Or',op('Greater',remain,half),op('And',op('Equal',remain,half),odd))
        rounded=op('Add',whole,cast(increment,T.INT64))
        return op('Where',neg,op('Neg',rounded),rounded)
    def clip(a,lo,hi,dtype=np.int64):
        return op('Clip',a,const(np.array(lo,dtype)),const(np.array(hi,dtype)))
    def qcdq(code,scale,zero,bits,signed,axis=None):
        dtype=np.int8 if signed else np.uint8
        zp=const(np.asarray(zero,dtype=dtype));sc=const(np.asarray(scale,dtype=np.float32))
        attrs={} if axis is None else {'axis':axis}
        value=op('DequantizeLinear',code,sc,zp,**attrs)
        q=op('QuantizeLinear',value,sc,zp,**attrs)
        lo,hi=(-2**(bits-1),2**(bits-1)-1) if signed else (0,2**bits-1)
        if bits<8:q=clip(q,lo,hi,dtype)
        return q,value,sc
    encoded={'input':'image_int8'}
    qparams={'input':(1/255,-128,8,True)}
    shapes={}
    for row in spec['nodes']:
        name=row['id'];inputs=[encoded[k] for k in row['inputs']]
        if row['op']=='upsample':
            value=op('Resize',inputs[0],const(np.array([],np.float32)),const(np.array([1,1,2,2],np.float32)),
                     mode='nearest',coordinate_transformation_mode='asymmetric',nearest_mode='floor')
        elif row['op']=='conv':
            inp=row['inputs'][0];si,zi,bi,signedi=qparams[inp]
            _,xdq,sc=qcdq(inputs[0],si,zi,bi,signedi)
            xcodes=op('Round',op('Div',xdq,sc))
            w=ref.weights[name].numpy().astype(np.int8)
            ws=ref.arrays[name+'__weight_scale']
            wz=const(np.zeros(len(ws),np.int8))
            wscale=const(ws)
            wf=const(w.astype(np.float32)*ws[:,None,None,None])
            wq=op('QuantizeLinear',wf,wscale,wz,axis=0)
            if row['weight_bits']<8:
                bits=row['weight_bits'];wq=clip(wq,-2**(bits-1),2**(bits-1)-1,np.int8)
            wdq=op('DequantizeLinear',wq,wscale,wz,axis=0)
            wc=op('Round',op('Div',wdq,const(ws[:,None,None,None])))
            bias=const(ref.arrays[name+'__bias'].astype(np.float32))
            pads=row['padding']*2
            acc=op('Conv',xcodes,wc,bias,strides=row['stride'],pads=pads,
                   dilations=row['dilation'],group=row['groups'])
            acc=cast(op('Round',acc),T.INT64)
            mult=const(np.asarray(row['multiplier'],np.int64)[None,:,None,None])
            shift=np.asarray(row['shift'],np.int64)[None,:,None,None]
            value=rne(op('Mul',acc,mult),shift)
        else:
            aligned=[]
            for x,z,m in zip(inputs,row['input_zero_points'],row['multiplier']):
                centered=op('Sub',cast(x,T.INT64),const(np.array(z,np.int64)))
                aligned.append(op('Mul',centered,const(np.array(m,np.int64))))
            value=rne(op('Add',*aligned),np.array(row['shift'],np.int64))
        bits=row['output_bits'];signed=row['output_signed']
        if row['op']!='upsample':
            lo,hi=(-2**(bits-1),2**(bits-1)-1) if signed else (0,2**bits-1)
            value=cast(clip(value,lo,hi),T.INT8 if signed else T.UINT8)
            value,_,_=qcdq(value,row['output_scale'],0,bits,signed)
        encoded[name]=value
        qparams[name]=(row['output_scale'],0,bits,signed)
    outputshapes=[[1,4,20,20],[1,10,20,20],[1,4,10,10],[1,10,10,10],[1,3,80,80]]
    for name,key in zip(OUTPUT_NAMES,spec['outputs']):
        nodes.append(H.make_node('Identity',[encoded[key]],[name]))
    graph=H.make_graph(nodes,'LDS-Net_QCDQ_integer_requantization',
        [H.make_tensor_value_info('image_int8',T.INT8,[1,3,320,320])],
        [H.make_tensor_value_info(n,T.INT8,s) for n,s in zip(OUTPUT_NAMES,outputshapes)],initial)
    model=H.make_model(graph,opset_imports=[H.make_opsetid('',17)],producer_name='LDS-Net reference')
    model.ir_version=9
    H.set_model_props(model,{'execution':'QCDQ reference with integer M/R; float Conv accumulates integer codes. Not native INT4 hardware.',
                            'input':'RGB NCHW p-128; letterbox114=-14; zero padding=-128',
                            'outputs':'five signed INT8 raw tensors; no NMS or decoding'})
    onnx.checker.check_model(model)
    forbidden={'Concat','BatchNormalization','Sigmoid','Softmax','TopK','NonMaxSuppression','ArgMax'}
    assert not (forbidden & {n.op_type for n in nodes})
    path=directory/'core_qcdq.onnx';onnx.save(model,path)
    return path


def verify_onnx(directory, images):
    import onnxruntime as ort
    directory=Path(directory);path=export_onnx(directory)
    session=ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
    ref=IntegerReference(directory)
    maxdiff=[0]*5
    for image in images:
        q=(image.cpu()*255).round().short().sub(128).to(torch.int8).numpy()
        a=session.run(None,{'image_int8':q});b=ref(q)
        for i,(u,v) in enumerate(zip(a,b)):
            maxdiff[i]=max(maxdiff[i],int(np.abs(u.astype(np.int16)-v.numpy().astype(np.int16)).max()))
    passed=max(maxdiff)<=1
    write_json(directory/'onnx_verification.json',{'passed':passed,'max_difference_lsb':maxdiff,
        'onnx_container_bytes':path.stat().st_size,'native_INT4_kernel':False})
    if not passed:raise AssertionError(f'ONNX reference mismatch: {maxdiff}')
    return path
