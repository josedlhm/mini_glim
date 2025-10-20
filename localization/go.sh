#!/usr/bin/env bash
# go.sh — Run GLIM, fetch trajectory, then convert + match to per-frame camera poses.
#
# Usage:
#   bash go.sh /data/outbox/2025-10-19_15-36-12
#
# Env overrides (optional):
#   GLIM_CONFIG=/abs/path/to/glim_config.yaml
#   TRAJ_KIND=traj_lidar   # or: odom_lidar | traj_imu | odom_imu

set -euo pipefail

RUN_DIR="${1:?usage: bash go.sh /path/to/RUN_DIR}"
GLIM_CONFIG="${GLIM_CONFIG:-/home/user/glim_config}"
TRAJ_KIND="${TRAJ_KIND:-traj_lidar}"     # default to loop-closed LiDAR trajectory per GLIM docs
DUMP_DIR="/tmp/dump"                     # GLIM's documented dump location

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$RUN_DIR/output"                # save artifacts next to the bag, not in the repo
mkdir -p "$OUT_DIR"

BAG_DIR="$RUN_DIR/livox"
CAMERAS_CSV="$RUN_DIR/cameras.csv"       # must exist: idx,cam_ts_ns,host_ts_ns

# Sanity: bag dir should look like a rosbag2 directory (metadata.yaml or *.mcap)
if [[ ! -d "$BAG_DIR" ]]; then
  echo "[!] Expected bag directory not found: $BAG_DIR" >&2
  exit 2
fi
if [[ ! -f "$BAG_DIR/metadata.yaml" ]] && ! compgen -G "$BAG_DIR/"'*.mcap' >/dev/null; then
  echo "[!] $BAG_DIR does not contain metadata.yaml or *.mcap" >&2
  exit 3
fi

echo "[*] Run dir     : $RUN_DIR"
echo "[*] Bag dir     : $BAG_DIR"
echo "[*] Output dir  : $OUT_DIR"
echo "[*] GLIM config : $GLIM_CONFIG"
echo "[*] Dump dir    : $DUMP_DIR"
echo "[*] Traj kind   : $TRAJ_KIND (traj_lidar|odom_lidar|traj_imu|odom_imu)"

# 0) Ensure a clean dump directory so we don't pick stale files
mkdir -p "$DUMP_DIR"
rm -rf "$DUMP_DIR"/* 2>/dev/null || true

# 1) Run GLIM (writes to /tmp/dump per docs)
echo "[*] Running GLIM..."
( cd "$REPO_ROOT" && bash "$REPO_ROOT/run_glim.sh" "$BAG_DIR" "$GLIM_CONFIG" )

# 2) Collect the trajectory from /tmp/dump as documented
TRAJ_SRC="$DUMP_DIR/${TRAJ_KIND}.txt"
TRAJ_DST="$OUT_DIR/traj_lidar.txt"   # standardized name for downstream steps

if [[ -s "$TRAJ_SRC" ]]; then
  cp -f "$TRAJ_SRC" "$TRAJ_DST"
  echo "[ok] copied $TRAJ_SRC -> $TRAJ_DST"
else
  echo "[!] Expected $TRAJ_SRC not found. Checking alternatives in $DUMP_DIR ..."
  copied=""
  for alt in traj_lidar odom_lidar traj_imu odom_imu; do
    if [[ -s "$DUMP_DIR/$alt.txt" ]]; then
      cp -f "$DUMP_DIR/$alt.txt" "$TRAJ_DST"
      echo "[ok] fallback: copied $DUMP_DIR/$alt.txt -> $TRAJ_DST"
      copied="yes"
      break
    fi
  done
  if [[ -z "$copied" ]]; then
    echo "[!] No trajectory file (traj_lidar/odom_lidar/traj_imu/odom_imu) found in $DUMP_DIR" >&2
    echo "    Contents:"; ls -l "$DUMP_DIR" || true
    exit 4
  fi
fi

# 2.5) cameras.csv must exist (created by your SVO extractor or earlier step)
if [[ ! -s "$CAMERAS_CSV" ]]; then
  echo "[!] Missing $CAMERAS_CSV (expected CSV with columns: idx,cam_ts_ns,host_ts_ns)" >&2
  exit 5
fi

# 3) Convert LiDAR->Camera + match to frames (single pass)
echo "[*] Converting + matching (LiDAR->Camera, then per-frame interpolation)..."
TMP_CFG="$(mktemp)"
cat > "$TMP_CFG" <<EOF
{
  "in_tum":       "$(printf '%s' "$TRAJ_DST")",
  "extrinsics":   "$(printf '%s' "$REPO_ROOT/config/extrinsics.json")",
  "cameras":      "$(printf '%s' "$CAMERAS_CSV")",
  "out_txt":      "$(printf '%s' "$OUT_DIR/poses_at_frames.txt")",
  "out_idxs":     "$(printf '%s' "$OUT_DIR/kept_indices.txt")",
  "out_rt_mm":    "$(printf '%s' "$OUT_DIR/poses_r_t_mm.txt")",
  "ts_col":       "cam_ts_ns"
}
EOF

# Python env
source ".venv/bin/activate"

# Run the combined convert+match script
python "$REPO_ROOT/lidar_tum_to_rt_mm.py" "$TMP_CFG"
rm -f "$TMP_CFG"

echo "[✓] Done."
echo "    traj_lidar        : $TRAJ_DST"
echo "    poses_at_frames   : $OUT_DIR/poses_at_frames.txt   # idx cam_ts_s x_mm y_mm z_mm qx qy qz qw"
echo "    kept_indices      : $OUT_DIR/kept_indices.txt"
echo "    poses_r_t_mm      : $OUT_DIR/poses_r_t_mm.txt      # idx t r11 r12 r13 tx_mm r21 r22 r23 ty_mm r31 r32 r33 tz_mm"
