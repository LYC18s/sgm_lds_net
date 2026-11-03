"""Fixed metric-distance color anchors; raw six-bit disparity is unchanged."""
import numpy as np

JET_RGB = np.array([
    [0, 0, 128],
    [0, 0, 144],
    [0, 0, 160],
    [0, 0, 176],
    [0, 0, 192],
    [0, 0, 208],
    [0, 0, 224],
    [0, 0, 240],
    [0, 0, 255],
    [0, 16, 255],
    [0, 32, 255],
    [0, 48, 255],
    [0, 64, 255],
    [0, 80, 255],
    [0, 96, 255],
    [0, 112, 255],
    [0, 132, 255],
    [0, 148, 255],
    [0, 164, 255],
    [0, 180, 255],
    [0, 196, 255],
    [0, 212, 255],
    [0, 228, 255],
    [0, 244, 255],
    [6, 255, 250],
    [22, 255, 234],
    [38, 255, 218],
    [54, 255, 202],
    [70, 255, 186],
    [86, 255, 170],
    [102, 255, 154],
    [118, 255, 138],
    [138, 255, 118],
    [154, 255, 102],
    [170, 255, 86],
    [186, 255, 70],
    [202, 255, 54],
    [218, 255, 38],
    [234, 255, 22],
    [250, 255, 6],
    [255, 244, 0],
    [255, 228, 0],
    [255, 212, 0],
    [255, 196, 0],
    [255, 180, 0],
    [255, 164, 0],
    [255, 148, 0],
    [255, 132, 0],
    [255, 112, 0],
    [255, 96, 0],
    [255, 80, 0],
    [255, 64, 0],
    [255, 48, 0],
    [255, 32, 0],
    [255, 16, 0],
    [255, 0, 0],
    [240, 0, 0],
    [224, 0, 0],
    [208, 0, 0],
    [192, 0, 0],
    [176, 0, 0],
    [160, 0, 0],
    [144, 0, 0],
    [128, 0, 0],
], dtype=np.uint8)

FOCAL_BASELINE_M_PX = 29.951004850458116
DISTANCE_ANCHORS_M = np.array([.5,1.,2.5,5.,10.,30.])
DISTANCE_ANCHOR_RGB = np.array([[255,0,0],[255,128,0],[255,255,0],
                               [0,255,0],[0,255,255],[0,0,255]],dtype=np.uint8)


def distance_to_rgb(distance_m):
    """Continuous, piecewise-linear RGB interpolation at fixed meter anchors."""
    distances=np.asarray(distance_m,dtype=float)
    channels=[np.interp(distances,DISTANCE_ANCHORS_M,DISTANCE_ANCHOR_RGB[:,i]) for i in range(3)]
    return np.rint(np.stack(channels,axis=-1)).astype(np.uint8)


DISPLAY_RGB = np.empty((64,3),dtype=np.uint8)
DISPLAY_RGB[0] = [0,0,128]
DISPLAY_RGB[1:] = distance_to_rgb(FOCAL_BASELINE_M_PX/np.arange(1,64,dtype=float))


def colorize_disparity(disparity):
    """Map integer disparities 0..63 to RGB; zero has no validity guarantee."""
    return DISPLAY_RGB[np.clip(np.asarray(disparity), 0, 63).astype(np.uint8)]
