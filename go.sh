#!/usr/bin/env bash
# Single entrypoint: pass the run directory (e.g., /data/outbox/2025-10-19_15-36-12)
#
# Usage:
#   bash go.sh /data/outbox/2025-10-19_15-36-12
#
# Env overrides (optional):
#   GLIM_CONFIG=/abs/path/to/glim_config.yaml

set -euo pipefail

RUN_DIR="${1:?usage: bash go.sh /path/to/RUN_DIR}"
GLIM_CONFIG="${GLIM_CONFIG:-/home/user/glim_config}"

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

# 1) Run GLIM (only if trajectory not present)
TRAJ="$OUT_DIR/traj_lidar.txt"
if [[ ! -s "$TRAJ" ]]; then
  echo "[*] Running GLIM..."
  # Run from OUT_DIR so GLIM writes outputs here deterministically
  ( cd "$OUT_DIR" && bash "$REPO_ROOT/run_glim.sh" "$BAG_DIR" "$GLIM_CONFIG" )
  if [[ ! -s "$TRAJ" ]]; then
    echo "[!] GLIM finished but $TRAJ not found. Check logs/output." >&2
    exit 4
  fi
else
  echo "[*] Reusing existing $TRAJ"
fi

# 2) Convert to row-major 3x4 [R|t] with translation in mm
echo "[*] Converting LiDAR -> Camera [R|t]_mm ..."
python3 "$REPO_ROOT/lidar_tum_to_rt_mm.py" "$REPO_ROOT/config/glim_mini.json"

echo "[✓] Done."
echo "    poses_r_t_mm: $OUT_DIR/poses_r_t_mm.txt"
