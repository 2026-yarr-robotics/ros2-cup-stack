#!/bin/bash
# Doosan M0609 MoveIt bringup - simulation/virtual mode.

set -e

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

echo "[SIM] DSR M0609 MoveIt bringup (mode=virtual)"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:=m0609 \
    mode:=virtual \
    host:=127.0.0.1 \
    port:=12345
