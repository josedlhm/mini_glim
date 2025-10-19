#!/usr/bin/env python3
"""
lidar_tum_to_rt_mm.py

Convert GLIM LiDAR TUM poses (t x y z qx qy qz qw) to row-major 3x4 [R|t]
for the CAMERA, with translation in millimeters.

Math:  T_w_c = T_w_l @ T_l_c

Config (default: config/glim_mini.json):
  - in_tum:     path to LiDAR TUM (meters; LiDAR in world)
  - extrinsics: path to LiDAR->Camera extrinsic (meters; q as xyzw)
  - out_rt_mm:  output file for 3x4 [R|t] rows with t in mm
"""
import sys, json
from pathlib import Path
import numpy as np
from math import isclose
from scipy.spatial.transform import Rotation as R

DEF_CFG = "config/glim_mini.json"

def load_config(cfg_path: Path):
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config not found: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())
    for k in ["in_tum", "out_rt_mm", "extrinsics"]:
        if k not in cfg:
            raise KeyError(f"Missing '{k}' in {cfg_path}")
    def rp(k): return Path(cfg[k]).expanduser().resolve()
    return {"in_tum": rp("in_tum"), "out_rt_mm": rp("out_rt_mm"), "extrinsics": rp("extrinsics")}

def pose_to_T(x, y, z, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3,:3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def T_to_rt_row_major_mm(T):
    Rm = T[:3,:3]
    t_mm = T[:3,3] * 1000.0
    Rt = np.hstack([Rm, t_mm.reshape(3,1)]).reshape(-1)
    return Rt

def main():
    cfg_path = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else Path(DEF_CFG).resolve()
    paths = load_config(cfg_path)

    # LiDAR->Camera extrinsic
    E = json.loads(paths["extrinsics"].read_text())
    t_lc = np.array([E["tx"], E["ty"], E["tz"]], dtype=float)
    q_lc = np.array([E["qx"], E["qy"], E["qz"], E["qw"]], dtype=float)
    qn = np.linalg.norm(q_lc)
    if not isclose(qn, 1.0, rel_tol=1e-6, abs_tol=1e-6):
        q_lc = q_lc / qn  # safe normalization
    T_lc = np.eye(4)
    T_lc[:3,:3] = R.from_quat(q_lc).as_matrix()
    T_lc[:3, 3] = t_lc

    in_tum = paths["in_tum"]
    if not in_tum.exists():
        raise FileNotFoundError(f"Input trajectory not found: {in_tum}")

    out_rt_mm = paths["out_rt_mm"]

    n = 0
    with open(in_tum, "r") as fin, open(out_rt_mm, "w") as fout:
        for line in fin:
            s = line.strip()
            if not s or s.startswith("#"): continue
            toks = s.split()
            if len(toks) < 8: continue  # skip non-TUM rows
            # TUM: t x y z qx qy qz qw
            x, y, z, qx, qy, qz, qw = map(float, toks[1:8])

            T_wl = pose_to_T(x, y, z, qx, qy, qz, qw)
            T_wc = T_wl @ T_lc

            Rt_mm = T_to_rt_row_major_mm(T_wc)
            fout.write(" ".join(f"{v:.9f}" for v in Rt_mm) + "\n")
            n += 1

    print(f"[ok] wrote {n} rows to {out_rt_mm}")

if __name__ == "__main__":
    main()
