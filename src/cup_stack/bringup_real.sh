#!/bin/bash
# Doosan M0609 MoveIt bringup - real robot mode.
# Usage: ./bringup_real.sh [ROBOT_IP]

set -e

ROBOT_IP=${1:-${ROBOT_IP:-192.168.1.100}}
ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

find_workspace_setup() {
    local dir="$SCRIPT_DIR"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/install/setup.bash" ]; then
            echo "$dir/install/setup.bash"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

WORKSPACE_SETUP=$(find_workspace_setup || true)
if [ -n "$WORKSPACE_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WORKSPACE_SETUP"
else
    echo "[WARN] workspace install/setup.bash not found. Run colcon build first."
fi

echo "[REAL] DSR M0609 MoveIt bringup (mode=real, host=${ROBOT_IP})"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:=m0609 \
    mode:=real \
    host:="${ROBOT_IP}" \
    port:=12345
