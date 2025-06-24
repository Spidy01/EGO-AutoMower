# EGO-AutoMower

Openmower adaptation for the Ego Z6 mower based on the work by MartinRob
and ClemensElflein.

## ROS Integration

Two ROS packages are provided:

* **esp32_serial_bridge** – communicates with the ESP32‑C6 over USB and
  publishes/subscribes raw RS485 frames.
* **z6_mower_bridge** – translates between the `open_mower_ros` topics
  and the RS485 protocol used by the mower. Unknown message mappings are
  marked with TODOs.

Use `ros/start_z6.sh` to launch the micro-ROS agent, the bridge node and the
`open_mower_ros` stack.

## Firmware

The firmware inside `firmware/esp32` runs micro-ROS and only forwards
complete RS485 frames between ROS and the bus. Translation is performed
on the host computer by `z6_mower_bridge`.

Additional digital I/O is exposed:

* `/blade_enable` (Bool) - turns the mower blades on or off.
* `/motion_enable` (Bool) - simulates the operator seat switch to allow motion.
* `/estop_state` (Bool) - publishes the emergency stop status (true when healthy).
* `/bus_enable` (Bool) - enables RS485 output and pulls the CC pin low for autonomous control.
