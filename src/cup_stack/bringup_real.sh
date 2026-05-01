#!/bin/bash
# Doosan M0609 MoveIt bringup - real robot mode.
# Usage: ./bringup_real.sh [ROBOT_IP]

set -e

ROBOT_IP=${1:-${ROBOT_IP:-192.168.1.100}}
ROS_DISTRO=${ROS_DISTRO:-humble}

source_if_exists() {
    local setup_file="$1"
    if [ -f "$setup_file" ]; then
        # shellcheck source=/dev/null
        source "$setup_file"
    fi
}

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source_if_exists "/home/ssu/ws_moveit/install/setup.bash"
source_if_exists "/home/ssu/ros2_ws/install/setup.bash"
source_if_exists "/home/ssu/install/setup.bash"

echo "[REAL] DSR M0609 MoveIt bringup (mode=real, host=${ROBOT_IP})"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:=m0609 \
    mode:=real \
    host:="${ROBOT_IP}" \
    port:=12345
