#!/usr/bin/env bash
# Single entrypoint: pass the run directory (e.g., /data/outbox/2025-10-19_15-36-12)
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
TRAJ_KIND="${TRAJ_KIND:-traj_lidar}"     # default to loop-closed LiDAR trajectory per docs
DUMP_DIR="/tmp/dump"                     # GLIM's documented dump location

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$REPO_ROOT/glim"
mkdir -p "$OUT_DIR"

BAG_DIR="$RUN_DIR/livox"

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

# 1) Run GLIM
echo "[*] Running GLIM..."
( cd "$OUT_DIR" && bash "$REPO_ROOT/run_glim.sh" "$BAG_DIR" "$GLIM_CONFIG" )

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

# 3) Convert to row-major 3x4 [R|t] with translation in mm
echo "[*] Converting LiDAR -> Camera [R|t]_mm ..."
python3 "$REPO_ROOT/lidar_tum_to_rt_mm.py" "$REPO_ROOT/config/glim_mini.json"

echo "[✓] Done."
echo "    poses_r_t_mm: $OUT_DIR/poses_r_t_mm.txt"
