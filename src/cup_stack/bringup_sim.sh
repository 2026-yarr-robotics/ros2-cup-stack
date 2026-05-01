#!/bin/bash
# Doosan M0609 MoveIt bringup - simulation/virtual mode.

set -e

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

echo "[SIM] DSR M0609 MoveIt bringup (mode=virtual)"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:=m0609 \
    mode:=virtual \
    host:=127.0.0.1 \
    port:=12345
