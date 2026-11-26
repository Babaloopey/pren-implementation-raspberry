#!/bin/bash
# Builds the mounted workspace (incremental, symlink install: Python edits need no rebuild)
# and runs the given command (default: the simulator).
set -e
source /opt/ros/jazzy/setup.bash
cd /ws
echo "[entrypoint] colcon build ..."
if ! colcon build --symlink-install --packages-skip robot_communication; then
    echo "[entrypoint] colcon build failed, see above" >&2
    exit 1
fi
source install/setup.bash
exec "$@"
