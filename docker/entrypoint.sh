#!/bin/bash
# Builds the mounted workspace (incremental, symlink install: Python edits need no rebuild)
# and runs the given command (default: the simulator).
#
# Hardware-in-the-loop (docker-compose.hil.yml): with RMW_IMPLEMENTATION=rmw_zenoh_cpp a Zenoh
# router is started that connects to the router on the Raspberry Pi (PI_HOST). All nodes in the
# container are Zenoh clients of that router, so everything goes through one outgoing TCP
# connection to the Pi (works through Docker Desktop's NAT).
set -e
source /opt/ros/jazzy/setup.bash
cd /ws
echo "[entrypoint] colcon build ..."
if ! colcon build --symlink-install --packages-skip robot_communication; then
    echo "[entrypoint] colcon build failed, see above" >&2
    exit 1
fi
source install/setup.bash

if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
    if [ -z "$PI_HOST" ]; then
        echo "[entrypoint] PI_HOST fehlt (IP-Adresse des Raspberry Pi)" >&2
        exit 1
    fi
    echo "[entrypoint] Zenoh-Router, verbindet zu tcp/${PI_HOST}:7447 ..."
    ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/${PI_HOST}:7447\"]" \
        ros2 run rmw_zenoh_cpp rmw_zenohd > /ws/logs/zenoh_router.log 2>&1 &
    sleep 2
fi
exec "$@"
