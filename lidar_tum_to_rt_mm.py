#!/usr/bin/env python3
"""
lidar_tum_to_rt_mm.py  (now: convert + match in one pass)

Reads:
  - LiDAR TUM (GLIM):    t x y z qx qy qz qw     (meters, seconds; LiDAR in world)
  - Extrinsics JSON:     {tx,ty,tz,qx,qy,qz,qw}  (meters; quaternion xyzw; LiDAR->Camera)
  - cameras.csv:         idx,cam_ts_ns,host_ts_ns

Computes per-frame CAMERA pose by:
  1) Interpolating LiDAR pose at each camera timestamp (linear + SLERP)
  2) Applying extrinsic: T_w_c(t) = T_w_l(t) @ T_l_c
  3) Converting translation to millimeters

Writes:
  - out_txt   : poses_at_frames.txt
      "idx cam_ts_s x_mm y_mm z_mm qx qy qz qw"
  - out_idxs  : kept_indices.txt
      each line: idx
  - out_rt_mm : poses_r_t_mm.txt  (matrix-friendly, aligned to kept frames)
      "idx t r11 r12 r13 tx_mm r21 r22 r23 ty_mm r31 r32 r33 tz_mm"

Config JSON (passed as argv[1]):
{
  "in_tum": "/abs/path/traj_lidar.txt",
  "extrinsics": "/abs/path/extrinsics.json",
  "cameras": "/abs/path/cameras.csv",
  "out_txt": "/abs/path/output/poses_at_frames.txt",
  "out_idxs": "/abs/path/output/kept_indices.txt",
  "out_rt_mm": "/abs/path/output/poses_r_t_mm.txt",
  "ts_col": "cam_ts_ns"   // optional (default: cam_ts_ns)
}

Notes:
- Units: input positions in meters; outputs positions in millimeters.
- Timestamps in outputs are seconds (float).
"""

import sys, json, csv
from pathlib import Path
import numpy as np
from math import isclose
from scipy.spatial.transform import Rotation as R, Slerp

DEF_CFG = "config/glim_mini.json"

# ---------- IO / config ----------

def _rp(cfg, k):
    return Path(cfg[k]).expanduser().resolve()

def load_config(cfg_path: Path):
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config not found: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())

    required = ["in_tum", "extrinsics", "cameras", "out_txt", "out_idxs", "out_rt_mm"]
    missing = [k for k in required if k not in cfg]
    if missing:
        raise KeyError(f"Missing required keys in {cfg_path}: {missing}. "
                       f"Expected keys: {required}")

    ts_col = cfg.get("ts_col", "cam_ts_ns")

    return {
        "in_tum": _rp(cfg, "in_tum"),
        "extrinsics": _rp(cfg, "extrinsics"),
        "cameras": _rp(cfg, "cameras"),
        "out_txt": _rp(cfg, "out_txt"),
        "out_idxs": _rp(cfg, "out_idxs"),
        "out_rt_mm": _rp(cfg, "out_rt_mm"),
        "ts_col": ts_col,
    }

# ---------- math helpers ----------

def pose_to_T(x, y, z, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def compose_T(Rm, t):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T

# ---------- loaders ----------

def load_lidar_tum(path: Path):
    ts, p_m, q_xyzw = [], [], []
    with open(path, "r") as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            a = s.split()
            if len(a) < 8:
                continue
            t = float(a[0])
            x, y, z = map(float, a[1:4])         # meters
            qx, qy, qz, qw = map(float, a[4:8])  # xyzw
            ts.append(t)
            p_m.append([x, y, z])
            q_xyzw.append([qx, qy, qz, qw])
    ts = np.asarray(ts, dtype=np.float64)
    order = np.argsort(ts)
    ts = ts[order]
    p_m = np.asarray(p_m, dtype=np.float64)[order]
    q_xyzw = np.asarray(q_xyzw, dtype=np.float64)[order]
    # normalize quats defensively
    q_xyzw = q_xyzw / np.clip(np.linalg.norm(q_xyzw, axis=1, keepdims=True), 1e-12, None)
    return ts, p_m, q_xyzw

def load_extrinsics(path: Path):
    E = json.loads(path.read_text())
    t = np.array([E["tx"], E["ty"], E["tz"]], dtype=float)          # meters
    q = np.array([E["qx"], E["qy"], E["qz"], E["qw"]], dtype=float) # xyzw
    qn = np.linalg.norm(q)
    if not isclose(qn, 1.0, rel_tol=1e-6, abs_tol=1e-6):
        q = q / max(qn, 1e-12)
    Rlc = R.from_quat(q).as_matrix()
    return Rlc, t

def load_frames(path: Path, ts_col: str):
    frames = []  # (idx:int, t_sec:float)
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        if ts_col not in rd.fieldnames:
            raise ValueError(f"{ts_col} not found in cameras.csv headers: {rd.fieldnames}")
        for row in rd:
            idx = int(row["idx"])
            ts_ns = int(row[ts_col])
            frames.append((idx, ts_ns * 1e-9))  # ns -> s
    return frames

# ---------- interpolation ----------

def interpolate_camera(ts_l, p_l_m, q_l_xyzw, Rlc, tlc_m, t_query):
    """
    Interpolate LiDAR pose at t_query, then transform to CAMERA:
        T_w_c(t) = T_w_l(t) @ T_l_c
    Returns (p_c_mm (3,), q_wc_xyzw (4,)) or None if out of range.
    """
    if not (ts_l[0] <= t_query <= ts_l[-1]):
        return None

    i = np.searchsorted(ts_l, t_query)
    i0 = max(0, min(len(ts_l) - 2, i - 1))
    i1 = i0 + 1
    t0, t1 = ts_l[i0], ts_l[i1]
    alpha = 0.0 if t1 == t0 else (t_query - t0) / (t1 - t0)
    alpha = float(np.clip(alpha, 0.0, 1.0))

    # LiDAR translation (linear, meters)
    p0, p1 = p_l_m[i0], p_l_m[i1]
    p_l_m_interp = (1.0 - alpha) * p0 + alpha * p1

    # LiDAR rotation (SLERP)
    rkey = R.from_quat(np.vstack([q_l_xyzw[i0], q_l_xyzw[i1]]))
    slerp = Slerp([t0, t1], rkey)
    R_wl = slerp([t_query]).as_matrix()[0]

    # Camera transform
    T_wl = compose_T(R_wl, p_l_m_interp)
    T_lc = compose_T(Rlc, tlc_m)
    T_wc = T_wl @ T_lc

    R_wc = T_wc[:3, :3]
    t_wc_m = T_wc[:3, 3]
    q_wc_xyzw = R.from_matrix(R_wc).as_quat()
    p_c_mm = t_wc_m * 1000.0
    return p_c_mm, q_wc_xyzw

# ---------- writers ----------

def write_poses_at_frames(path: Path, idx_list, t_list_s, p_mm_list, q_xyzw_list):
    with open(path, "w") as f:
        for idx, t_s, p_mm, q in zip(idx_list, t_list_s, p_mm_list, q_xyzw_list):
            f.write(f"{idx:d} {t_s:.9f} "
                    f"{p_mm[0]:.6f} {p_mm[1]:.6f} {p_mm[2]:.6f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")

def write_indices(path: Path, idx_list):
    with open(path, "w") as f:
        for idx in idx_list:
            f.write(f"{idx}\n")

def write_rt_mm_with_idx_time(path: Path, idx_list, t_list_s, p_mm_list, q_xyzw_list):
    with open(path, "w") as f:
        for idx, t_s, p_mm, q in zip(idx_list, t_list_s, p_mm_list, q_xyzw_list):
            Rm = R.from_quat(q).as_matrix()
            Rt = np.hstack([Rm, p_mm.reshape(3,1)]).reshape(-1)
            f.write(f"{idx:d} {t_s:.9f} " + " ".join(f"{v:.9f}" for v in Rt) + "\n")

# ---------- main ----------

def main():
    cfg_path = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else Path(DEF_CFG).resolve()
    P = load_config(cfg_path)

    # Sanity
    if not P["in_tum"].exists():
        raise FileNotFoundError(f"Input LiDAR TUM not found: {P['in_tum']}")
    if not P["cameras"].exists():
        raise FileNotFoundError(f"cameras.csv not found: {P['cameras']}")
    if not P["extrinsics"].exists():
        raise FileNotFoundError(f"Extrinsics JSON not found: {P['extrinsics']}")

    ts_l, p_l_m, q_l_xyzw = load_lidar_tum(P["in_tum"])
    Rlc, tlc_m = load_extrinsics(P["extrinsics"])
    frames = load_frames(P["cameras"], P["ts_col"])

    kept_idx, kept_t_s, kept_p_mm, kept_q = [], [], [], []

    tmin, tmax = float(ts_l[0]), float(ts_l[-1])
    for idx, t_s in frames:
        inter = interpolate_camera(ts_l, p_l_m, q_l_xyzw, Rlc, tlc_m, t_s)
        if inter is None:
            continue
        p_mm, q = inter
        kept_idx.append(idx)
        kept_t_s.append(t_s)
        kept_p_mm.append(p_mm)
        kept_q.append(q)

    # Ensure output dirs exist
    P["out_txt"].parent.mkdir(parents=True, exist_ok=True)
    P["out_idxs"].parent.mkdir(parents=True, exist_ok=True)
    P["out_rt_mm"].parent.mkdir(parents=True, exist_ok=True)

    # Write files
    write_poses_at_frames(P["out_txt"], kept_idx, kept_t_s, kept_p_mm, kept_q)
    write_indices(P["out_idxs"], kept_idx)
    write_rt_mm_with_idx_time(P["out_rt_mm"], kept_idx, kept_t_s, kept_p_mm, kept_q)

    print(f"[ok] Kept {len(kept_idx)} frames within LiDAR pose span [{tmin:.6f}, {tmax:.6f}]")
    print(f"[ok] Wrote:\n  - poses_at_frames : {P['out_txt']}\n  - kept_indices   : {P['out_idxs']}\n  - poses_r_t_mm   : {P['out_rt_mm']}")

if __name__ == "__main__":
    main()
