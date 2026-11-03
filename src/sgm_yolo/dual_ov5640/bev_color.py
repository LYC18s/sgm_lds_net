"""Sparse body-color estimates from the owned, unoverlaid RGB3 NN input."""
import numpy as np

RGB_CAPABILITY=0x52474231
COLOR_CAPABILITY=0x434f4c31
COLOR_NAMES=('unknown','white','silver','black','red','blue','green','yellow','orange','brown')
COLOR_RGB=((140,151,163),(220,224,230),(140,151,163),(38,43,50),(190,42,36),
    (40,83,180),(42,125,82),(225,189,35),(232,105,32),(126,74,48))
PAINT_INDICES=(1,2,3,4,5,6,8,9)


def paint_palette(original,color_class):
    """Only warm body-paint indices change; glass/tires retain source colors."""
    palette=np.array(original,dtype=np.uint8).copy();base=np.array(COLOR_RGB[color_class],dtype=np.int32)
    for index,add,num,den in ((1,32,1,1),(2,16,1,1),(3,8,1,1),(4,0,1,1),
            (5,0,3,4),(6,0,1,2),(8,0,1,3),(9,0,1,5)):
        palette[index]=np.clip(base*num//den+add,0,255)
    return palette


def estimate_color(codes,box):
    unknown=dict(color_class=0,name='unknown',confidence=0.,rgb=list(COLOR_RGB[0]),samples=0)
    if np.shape(codes)!=(320,320,3):raise ValueError('Expected owned 320x320 RGB3 input')
    if codes.dtype not in (np.dtype('int8'),np.dtype('uint8')):raise ValueError('Expected q=p-128 input bytes')
    x1,y1,x2,y2=(float(box[k]) for k in ('x1','y1','x2','y2'))
    if not np.all(np.isfinite([x1,y1,x2,y2])) or x2<=x1 or y2<=y1:return unknown
    if (y2-y1)*2/3<24 or (x2-x1)*2/3<12:return unknown
    # Pixel centers of the exact 480->320 half-pixel crop/resize. Use lower
    # central body regions, excluding the top glass and extreme bbox edges.
    u=x1+(x2-x1)*np.linspace(.16,.84,20)
    v=y1+(y2-y1)*np.linspace(.32,.78,20)
    xs=np.unique(np.rint((u-80+.5)*2/3-.5).astype(int));ys=np.unique(np.rint((v+.5)*2/3-.5).astype(int))
    xs=xs[(xs>=0)&(xs<320)];ys=ys[(ys>=0)&(ys<320)]
    if len(xs)<4 or len(ys)<4:return unknown
    data=codes.view(np.int8)[ys[:,None],xs[None,:]].astype(np.int16).reshape(-1,3)+128
    data=data[(data.max(1)<250)&(data.max(1)>8)]
    if len(data)<16:return unknown
    rgb=data.astype(float);mx=rgb.max(1);mn=rgb.min(1);delta=mx-mn
    sat=np.divide(delta,mx,out=np.zeros_like(mx),where=mx>0)
    hue=np.zeros(len(rgb));colored=delta>0
    for channel in range(3):
        hit=colored&(rgb.argmax(1)==channel)
        hue[hit]=((rgb[hit,(channel+1)%3]-rgb[hit,(channel+2)%3])/delta[hit]+2*channel)*60
    hue%=360
    labels=np.full(len(rgb),2,dtype=int)
    warm=(rgb[:,0]>rgb[:,1]+18)&(rgb[:,1]>rgb[:,2]+4)
    neutral=(sat<.33)|(delta<30)
    labels[neutral&(mx>=180)]=1
    labels[neutral&(mx<105)]=3
    labels[mx<60]=3
    chromatic=(~neutral|warm)&(mx>=60)
    labels[chromatic&((hue<18)|(hue>=345))]=4
    labels[chromatic&(hue>=18)&(hue<45)]=8
    labels[chromatic&(hue>=45)&(hue<75)]=7
    labels[chromatic&(hue>=75)&(hue<170)]=6
    labels[chromatic&(hue>=170)&(hue<285)]=5
    # Cyan/blue illumination on silver paint and glass is not blue car paint.
    labels[(labels==5)&(rgb[:,2]<rgb[:,1]+18)]=2
    labels[chromatic&(hue>=285)&(hue<345)]=4
    labels[chromatic&(hue<50)&warm&(mx<185)]=9
    counts=np.bincount(labels,minlength=10);weights=np.ones(len(labels))
    # Coherent colored body patches can be a minority beside neutral glass.
    # A few isolated tail lights (<12%) cannot recolor a neutral car.
    for color in range(4,10):
        if counts[color]>=max(16,.12*len(labels)):weights[labels==color]=6
    votes=np.bincount(labels,weights=weights,minlength=10)
    winner=int(votes.argmax());confidence=float(votes[winner]/weights.sum())
    if confidence<.38:return dict(unknown,samples=len(labels),confidence=confidence)
    representative=np.median(data[labels==winner],axis=0).astype(int).tolist()
    return dict(color_class=winner,name=COLOR_NAMES[winner],confidence=confidence,rgb=representative,samples=len(labels))


def attach_colors(detections,codes,frame):
    result=[]
    for value in detections:
        box=dict(value)
        if box.get('category') in (2,4,5):
            box['vehicle_color']=dict(estimate_color(codes,box),source_frame=int(frame))
        result.append(box)
    return result
