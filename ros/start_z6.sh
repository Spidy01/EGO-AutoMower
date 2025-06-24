#!/bin/bash
# Start micro-ROS agent, Z6 mower translation node and OpenMower stack

# source mower configuration if available
CONFIG_FILE="$(dirname $0)/../lib/Open_Mower_Ros/src/open_mower/config/mower_config.sh"
[ -f "$CONFIG_FILE" ] && source "$CONFIG_FILE"

micro_ros_agent serial --dev ${PORT:-/dev/ttyUSB0} &
AGENT_PID=$!

rosrun z6_mower_bridge z6_mower_node.py &
BRIDGE_PID=$!

# launch the original open_mower_ros nodes
roslaunch open_mower open_mower.launch &
OM_PID=$!

wait $AGENT_PID $BRIDGE_PID $OM_PID
