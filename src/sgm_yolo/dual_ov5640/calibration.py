"""Capture static stereo checkerboards and export this fixed rig's calibration.

This operates on RGB snapshots for calibration. It does not replace or reroute
the live PL SGM input, and exporting maps does not program a PL rectifier.
"""
import json
from pathlib import Path

import numpy as np


class StereoCalibration:
    def __init__(self, camera, directory="stereo_calibration", pattern=(9,6), square_mm=25.0):
        if not getattr(camera, "right_rgb_available", True):
            raise RuntimeError("New stereo RGB capture needs the archived dual-RGB hardware package; this compact bitstream retains the installed fixed calibration only.")
        import cv2
        self.cv2=cv2
        self.camera=camera
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.pattern=tuple(pattern)
        self.square_mm=float(square_mm)
        self.size=(640,480)
        self.order=tuple(getattr(camera,"physical_camera_order",(0,1)))
        obj=np.zeros((self.pattern[0]*self.pattern[1],3),np.float32)
        obj[:,:2]=np.mgrid[0:self.pattern[0],0:self.pattern[1]].T.reshape(-1,2)*self.square_mm
        self.object_points=obj
        self.left_points=[];self.right_points=[]
        meta=self.directory/"session.json"
        config={"pattern":list(self.pattern),"square_mm":self.square_mm,"size":list(self.size),"logical_to_physical_camera":list(self.order)}
        if meta.exists() and json.loads(meta.read_text(encoding="utf-8"))!=config:
            raise ValueError("Existing session has different board dimensions or camera order; choose another directory")
        meta.write_text(json.dumps(config,indent=2),encoding="utf-8")
        for file in sorted(self.directory.glob("corners_*.npz")):
            with np.load(str(file)) as data:
                self.left_points.append(data["left"].reshape(-1,1,2));self.right_points.append(data["right"].reshape(-1,1,2))

    def add_pair(self):
        """Call after the checkerboard is still and visible in both cameras."""
        cv2=self.cv2
        if not self.camera.running: raise RuntimeError("Start camera first")
        images=(self.camera.frame(0),self.camera.frame(1))
        found=[]
        for rgb in images:
            gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
            ok,corners=cv2.findChessboardCorners(gray,self.pattern,cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                return {"saved":False,"pairs":len(self.left_points),"reason":"All 9x6 inner corners must be visible in BOTH views; hold board still, avoid reflections"}
            corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
            found.append(np.asarray(corners,dtype=np.float32).reshape(-1,1,2))
        for a,b in zip(self.left_points,self.right_points):
            if max(np.linalg.norm(found[0]-a,axis=2).mean(),np.linalg.norm(found[1]-b,axis=2).mean())<8:
                return {"saved":False,"pairs":len(self.left_points),"reason":"Pose duplicates an existing pair; change board position/distance/tilt"}
        numbers=[int(p.stem.split('_')[-1]) for p in self.directory.glob("corners_*.npz")]
        number=max(numbers,default=-1)+1
        for name,rgb in zip(("left","right"),images):
            target=self.directory/("%s_%03d.png"%(name,number))
            if target.exists(): raise FileExistsError("Preserve previous capture: "+str(target))
            if not cv2.imwrite(str(target),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):
                raise IOError("Failed to save capture: "+str(target))
        np.savez_compressed(str(self.directory/("corners_%03d.npz"%number)),left=found[0],right=found[1])
        self.left_points.append(found[0]);self.right_points.append(found[1])
        return {"saved":True,"pairs":len(self.left_points),"target_pairs":"20-30 varied poses covering center, edges and different distances"}

    def solve(self):
        """Export real lens/extrinsic parameters and remap tables for PL design."""
        cv2=self.cv2
        if len(self.left_points)<12: raise ValueError("At least 12 varied valid pairs required; collect 20-30")
        objects=[self.object_points.copy() for _ in self.left_points]
        rms_l,k_l,d_l,_,_=cv2.calibrateCamera(objects,self.left_points,self.size,None,None)
        rms_r,k_r,d_r,_,_=cv2.calibrateCamera(objects,self.right_points,self.size,None,None)
        rms,k_l,d_l,k_r,d_r,rotation,translation,essential,fundamental=cv2.stereoCalibrate(
            objects,self.left_points,self.right_points,k_l,d_l,k_r,d_r,self.size,
            criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-6),flags=cv2.CALIB_FIX_INTRINSIC)
        r_l,r_r,p_l,p_r,q,roi_l,roi_r=cv2.stereoRectify(k_l,d_l,k_r,d_r,self.size,rotation,translation,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
        lx,ly=cv2.initUndistortRectifyMap(k_l,d_l,r_l,p_l,self.size,cv2.CV_32FC1)
        rx,ry=cv2.initUndistortRectifyMap(k_r,d_r,r_r,p_r,self.size,cv2.CV_32FC1)
        residuals=[]
        per_view=[]
        for left,right in zip(self.left_points,self.right_points):
            a=cv2.undistortPoints(left,k_l,d_l,R=r_l,P=p_l)
            b=cv2.undistortPoints(right,k_r,d_r,R=r_r,P=p_r)
            errors=(a[:,:,1]-b[:,:,1]).ravel()
            residuals.extend(errors.tolist());per_view.append(float(np.sqrt(np.mean(errors**2))))
        vertical_rms=float(np.sqrt(np.mean(np.array(residuals)**2)))
        horizontal=bool(abs(p_r[0,3])>abs(p_r[1,3]))
        positive_disparity=bool(horizontal and p_r[0,3]<0)
        yy=np.indices((480,640),dtype=np.float32)[0]
        def displacement(mx,my):
            valid=(mx>=0)&(mx<639)&(my>=0)&(my<479)
            diff=(my-yy)[valid]
            return [float(diff.min()),float(diff.max())] if diff.size else None
        report={
            "pairs":len(objects),"image_size":list(self.size),"square_mm":self.square_mm,
            "logical_to_physical_camera":list(self.order),
            "intrinsic_rms_left":rms_l,"intrinsic_rms_right":rms_r,"stereo_rms":rms,
            "rectified_vertical_rms":vertical_rms,"rectified_vertical_rms_per_pair":per_view,
            "rectification_axis":"horizontal" if horizontal else "vertical",
            "positive_disparity_geometry":positive_disparity,
            "quality_check_passed":bool(vertical_rms<1 and rms<1.5 and positive_disparity),
            "baseline_mm":float(np.linalg.norm(translation)),
            "left_source_row_displacement_min_max":displacement(lx,ly),
            "right_source_row_displacement_min_max":displacement(rx,ry),
            "camera_matrix_left":k_l.tolist(),"camera_matrix_right":k_r.tolist(),
            "distortion_left":d_l.tolist(),"distortion_right":d_r.tolist(),
            "rotation":rotation.tolist(),"translation_mm":translation.tolist(),
            "R_left":r_l.tolist(),"R_right":r_r.tolist(),"P_left":p_l.tolist(),"P_right":p_r.tolist(),"Q":q.tolist(),
            "valid_roi_left":list(roi_l),"valid_roi_right":list(roi_r),
            "pl_rectification_installed":False,
            "next_step":"Use these real maps to size and implement a live-stream PL rectifier; present bitstream corrects order only. Do not feed remapped DDR snapshots into SGM.",
        }
        (self.directory/"calibration.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        np.savez_compressed(str(self.directory/"rectification_maps.npz"),left_x=lx,left_y=ly,right_x=rx,right_y=ry)
        return report

    def preview(self):
        """Rectified RGB preview for validation only; SGM remains live in PL."""
        cv2=self.cv2
        with np.load(str(self.directory/"rectification_maps.npz")) as maps:
            left=cv2.remap(self.camera.frame(0),maps["left_x"],maps["left_y"],cv2.INTER_LINEAR)
            right=cv2.remap(self.camera.frame(1),maps["right_x"],maps["right_y"],cv2.INTER_LINEAR)
        canvas=np.concatenate((left,right),axis=1)
        for row in range(0,480,40): canvas[row:row+1]=[0,255,0]
        return canvas
