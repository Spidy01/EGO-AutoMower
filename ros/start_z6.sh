#!/bin/bash
# Start micro-ROS agent and Z6 mower translation node
micro_ros_agent serial --dev ${PORT:-/dev/ttyUSB0} &
rosrun z6_mower_bridge z6_mower_node.py &
wait
