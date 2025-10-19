#!/usr/bin/env bash
# Usage:
#   bash run_glim.sh /path/to/bag_dir [/path/to/glim_config]

set -eo pipefail
BAG_DIR="${1:?bag_dir (folder that contains the rosbag2 files)}"
GLIM_CONFIG="${2:-/home/user/glim_config}"

# ROS2 + Livox env

source /opt/ros/humble/setup.sh
source ~/ws_livox/install/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

echo "[*] Running GLIM..."
echo "    Bag:     $BAG_DIR"
echo "    Config:  $GLIM_CONFIG"

ros2 run glim_ros glim_rosbag "$BAG_DIR" --ros-args -p config_path:="$GLIM_CONFIG"

echo "[✓] GLIM finished successfully."
