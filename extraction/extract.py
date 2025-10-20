#!/usr/bin/env python3
"""
Module: extract_svo2.py

Extracts RGB, depth, and writes per-frame timestamps to cameras.csv
from an SVO/SVO2 using ZED SDK 5.x in playback mode.
"""
import os, csv
import cv2
import numpy as np
import pyzed.sl as sl

def extract_svo2(
    svo_path: str,
    output_dir: str,
    max_frames: int = None
) -> None:
    """
    Extracts up to `max_frames` frames from an SVO/SVO2 file, saving:
      - Left RGB images (lossless PNG)
      - Depth maps (full-precision .npy)
      - Frame timestamps (cameras.csv: idx, cam_ts_ns, host_ts_ns)

    Parameters:
        svo_path:    Path to input .svo or .svo2 file
        output_dir:  Directory where outputs are created
        max_frames:  Maximum number of frames to extract (None = all frames)
    """
    # Prepare output dirs
    os.makedirs(output_dir, exist_ok=True)
    img_dir   = os.path.join(output_dir, "images")
    depth_dir = os.path.join(output_dir, "depth")
    os.makedirs(img_dir,   exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    # 1) Initialize ZED in playback mode
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_path)
    init_params.svo_real_time_mode   = False
    init_params.depth_mode           = sl.DEPTH_MODE.NEURAL_PLUS
    init_params.coordinate_units     = sl.UNIT.MILLIMETER
    init_params.camera_resolution    = sl.RESOLUTION.HD1200
    init_params.coordinate_system    = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.camera_fps           = 30

    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open SVO/SVO2: {svo_path}")

    # 2) Enable positional tracking (same as before; depth extraction benefits from good motion)
    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.enable_imu_fusion = True
    tracking_params.mode              = sl.POSITIONAL_TRACKING_MODE.GEN_3
    if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
        zed.close()
        raise RuntimeError("Could not enable positional tracking (GEN_3 + IMU fusion)")

    # 3) Prepare Mats
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()

    # 4) Open cameras.csv for timestamps
    cams_csv = os.path.join(output_dir, "cameras.csv")
    with open(cams_csv, "w", newline="") as fcams:
        cams_writer = csv.writer(fcams)
        cams_writer.writerow(["idx","cam_ts_ns","host_ts_ns"])

        # 5) Frame loop (unchanged logic, minus poses.csv)
        total = zed.get_svo_number_of_frames()
        limit = total if max_frames is None else min(total, max_frames)
        frame_idx = 0

        while frame_idx < limit:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                break

            # Retrieve image & depth (as before)
            zed.retrieve_image(left_mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            # Timestamps (ns)
            cam_ts_ns  = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            host_ts_ns = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds()
            cams_writer.writerow([frame_idx, int(cam_ts_ns), int(host_ts_ns)])

            # Save RGB
            img_rgba = left_mat.get_data()
            img_bgr  = cv2.cvtColor(img_rgba, cv2.COLOR_BGRA2BGR)
            cv2.imwrite(
                os.path.join(img_dir, f"img_{frame_idx:06d}.png"),
                img_bgr,
                [cv2.IMWRITE_PNG_COMPRESSION, 0]
            )

            # Save depth (float32 mm)
            depth_data = depth_mat.get_data()
            np.save(os.path.join(depth_dir, f"depth_{frame_idx:06d}.npy"), depth_data)

            frame_idx += 1

    # Cleanup
    zed.close()
    print(f"✅ Extracted {frame_idx} frames (limit={limit}) to '{output_dir}'")
