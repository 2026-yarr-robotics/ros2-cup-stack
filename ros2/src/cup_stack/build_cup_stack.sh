#!/bin/bash
# Build this independent ROS 2 workspace.

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

find_workspace_root() {
    local dir="$SCRIPT_DIR"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/src/cup_stack/package.xml" ]; then
            echo "$dir"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

WORKSPACE_ROOT=$(find_workspace_root)

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "$WORKSPACE_ROOT"
colcon build --symlink-install "$@"
