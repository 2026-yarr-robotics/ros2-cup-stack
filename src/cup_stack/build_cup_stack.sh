#!/bin/bash
# Build this independent ROS 2 workspace.

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "$WORKSPACE_ROOT"
colcon build --symlink-install "$@"
