# ESP32 Serial Bridge

This package provides a simple ROS node for communicating with an ESP32 C6
microcontroller over a USB serial connection. Other parts of the OpenMower
system can publish commands to the `mower_control` topic and receive status
from the `mower_status` topic.

The actual mower control logic should be implemented on the ESP32. This node
only forwards string messages between ROS and the serial port.
