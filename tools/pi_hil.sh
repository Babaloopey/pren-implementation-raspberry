#!/bin/bash
# Raspberry Pi side of hardware-in-the-loop (variant B): the real robot nodes (camera, microphone,
# GPIO, recognition, state_coordinator, command_coordinator) without microcontroller; the laptop
# simulates the microcontroller and the field (start_simulator_hil.cmd <PI-IP> on the laptop).
#
#   tools/pi_hil.sh                    # everything
#   tools/pi_hil.sh gestures:=false    # extra arguments go to robot.launch.py
#
# Needs: sudo apt install ros-jazzy-rmw-zenoh-cpp   and a built workspace (colcon build).
# The laptop connects to TCP port 7447 of this Pi (same network, no client isolation in the WLAN).
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=17                 # same as the simulator container

ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
ROUTER=$!
trap 'kill $ROUTER 2>/dev/null' EXIT
sleep 2
echo "Zenoh-Router laeuft. IP-Adressen dieses Pi (fuer start_simulator_hil.cmd auf dem Laptop):"
hostname -I
# odom_source:=mcu: MOVE/TURN finish by the odometry the (virtual) microcontroller reports
# bones:=false: in the HIL the robot body is simulated, bones (detection + contact) come from the
# simulated field on the laptop. tools/pi_hil.sh bones:=true starts the real bone_detector anyway.
ros2 launch robot_core robot.launch.py odom_source:=mcu bones:=false "$@"
