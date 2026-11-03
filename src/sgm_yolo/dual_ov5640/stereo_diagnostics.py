"""Board probes separating disparity data, stereo geometry and HDMI output."""
import json
import time
from pathlib import Path

import numpy as np


def gray_statistics(gray):
    gray = np.asarray(gray, dtype=np.uint8)
    # Only 64 values are legal: RTL maps d to (d<<2)|(d>>4).
    legal = (gray & 3) == (gray >> 6)
    hist = np.bincount((gray >> 2).ravel(), minlength=64)
    return {
        "shape": list(gray.shape),
        "gray_levels": int(np.unique(gray).size),
        "invalid_gray_percent": float(100 * (1-legal.mean())),
        "zero_percent": float(100*(gray==0).mean()),
        "max_disparity_percent": float(100*(gray==255).mean()),
        "disparity_histogram": hist.tolist(),
    }


def stereo_geometry(left, right):
    """Sparse signed offsets; requires static textured scene for useful results.

    Images copied from RGB buffers are not the exact SGM input frame pair.
    Estimates diagnose geometry only; they are not calibration parameters.
    """
    try:
        import cv2
    except ImportError:
        return {"available": False, "reason": "OpenCV unavailable; original RGB frames saved for analysis"}
    l = cv2.cvtColor(np.asarray(left), cv2.COLOR_RGB2GRAY)
    r = cv2.cvtColor(np.asarray(right), cv2.COLOR_RGB2GRAY)
    orb = cv2.ORB_create(nfeatures=1800, fastThreshold=10)
    kl, dl = orb.detectAndCompute(l, None)
    kr, dr = orb.detectAndCompute(r, None)
    if dl is None or dr is None:
        return {"available": True, "matches": 0, "reason": "Insufficient texture"}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(dl, dr, k=2)
    reverse = matcher.knnMatch(dr, dl, k=2)
    back = {p[0].queryIdx: p[0].trainIdx for p in reverse if len(p)==2 and p[0].distance < .72*p[1].distance}
    matches = [p[0] for p in forward if len(p)==2 and p[0].distance < .72*p[1].distance and back.get(p[0].trainIdx)==p[0].queryIdx]
    if len(matches) < 8:
        return {"available": True, "matches": len(matches), "reason": "Too few distinctive reciprocal matches"}
    pl = np.float32([kl[m.queryIdx].pt for m in matches])
    pr = np.float32([kr[m.trainIdx].pt for m in matches])
    _, mask = cv2.findFundamentalMat(pl, pr, cv2.FM_RANSAC, 1.5, .995)
    if mask is None or int(mask.sum()) < 8:
        return {"available": True, "matches": len(matches), "reason": "No reliable epipolar fit"}
    pl, pr = pl[mask.ravel()!=0], pr[mask.ravel()!=0]
    dx, dy = pl[:,0]-pr[:,0], pr[:,1]-pl[:,1]
    valid = (dx>=0) & (dx<=63) & (np.abs(dy)<=2)
    return {
        "available": True, "matches": len(matches), "inliers": len(dx),
        "signed_disparity_definition": "left_x - right_x",
        "signed_disparity_median": float(np.median(dx)),
        "signed_disparity_p10_p90": np.percentile(dx,[10,90]).tolist(),
        "vertical_offset_median": float(np.median(dy)),
        "vertical_abs_p90": float(np.percentile(np.abs(dy),90)),
        "negative_disparity_percent": float(100*(dx<0).mean()),
        "over_63_disparity_percent": float(100*(dx>63).mean()),
        "outside_current_search_percent": float(100*(~valid).mean()),
        "points_left": pl.tolist(), "points_right": pr.tolist(),
        "note": "Static scene only; unsynchronized RGB snapshots, not exact SGM pair or stereo calibration",
    }


def capture_report(camera, directory="sgm_gray_report", settle_seconds=3):
    """Save original arrays/images and observe sticky DMA errors after clear."""
    if not camera.running:
        raise RuntimeError("Start camera in this Notebook kernel first")
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    channels = list(camera.write_channels) + [camera.disparity_write, camera.disparity_read]
    before = [hex(c.status()) for c in channels]
    # Read-only flags may survive acknowledgement depending on VDMA options.
    for c in channels:
        c.mmio.write(0x34 if c.direction=="write" else 0x04, 0xC990)
    immediate = [hex(c.status()) for c in channels]
    time.sleep(settle_seconds)
    after = [hex(c.status()) for c in channels]
    left = camera.frame(0)
    right = camera.frame(1) if getattr(camera, "right_rgb_available", True) else None
    gray = camera.disparity(visualization=True)
    buffers = []
    for f in camera.disparity_frames:
        camera._invalidate(f)
        buffers.append(np.array(f))
    report = {
        "driver_build": "geometry/data/display probe v1",
        "dma_channels": (["left_capture", "right_capture"] if right is not None else ["left_capture"]) + ["disparity_write", "disparity_read"],
        "status_before": before, "status_immediately_after_clear": immediate, "status_after_wait": after,
        "gray": gray_statistics(gray),
        "gray_all_buffers": [gray_statistics(f) for f in buffers],
        "geometry": stereo_geometry(left,right) if right is not None else {"available": False, "reason": "Right RGB DDR removed; fixed calibration still active in PL"},
        "limitation": "Copies from live buffers can cross frame boundaries; use a stationary scene. No exact hardware/CPU pixel comparison is claimed.",
    }
    arrays = dict(left=left,gray=gray,gray_buffers=np.stack(buffers))
    if right is not None: arrays["right"] = right
    np.savez_compressed(str(output/"frames.npz"),**arrays)
    try:
        from PIL import Image
        for name,array in (("left",left),("right",right),("disparity",gray)):
            if array is None: continue
            Image.fromarray(array).save(str(output/(name+".png")))
    except ImportError:
        pass
    (output/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    return report


def test_pattern():
    """Sixteen vertical gray bands, frame border and three horizontal bars."""
    y,x=np.indices((480,640))
    image=((x//40)*17).astype(np.uint8)
    image[220:224]=255
    image[300:304]=0
    image[380:384]=255
    image[:4]=255;image[-4:]=255;image[:,:4]=255;image[:,-4:]=255
    return image


def show_display_pattern(camera, duration=10):
    """Temporarily freeze SGM result DMA; RGB remains live. Always restore.

    Every disparity buffer receives the same standard image. This tests the
    result DDR -> MM2S -> compositor -> HDMI branch and bypasses SGM output.
    It does not redirect camera inputs through DDR or reprogram the FPGA.
    """
    if not camera.running:
        raise RuntimeError("Start camera first")
    addresses=tuple(camera._device_address(f) for f in camera.disparity_frames)
    camera.disparity_write.stop()
    try:
        deadline=time.monotonic()+1
        while not (camera.disparity_write.status() & 1):
            if time.monotonic()>=deadline:
                raise TimeoutError("Disparity writer did not halt; no buffers overwritten")
            time.sleep(.005)
        image=test_pattern()
        for f in camera.disparity_frames:
            f[:]=image;camera._flush(f)
        print("HDMI right panel: 16 straight vertical palette bands, three straight red horizontal bars, complete red border. RGB stays live.")
        time.sleep(duration)
    finally:
        # Re-arm against the next SOF; the live SGM path rejects partial input.
        camera.disparity_write.configure(addresses,640,480,640)
    print("SGM result capture restored; allow a few frames to replace the pattern.")
