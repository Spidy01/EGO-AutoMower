# ESP32 C6 Firmware

This firmware acts as a micro‑ROS node running on an ESP32‑C6.  It connects to a
micro‑ROS agent on the main computer via USB and publishes/subscribes to the
topics used by the **open_mower_ros** stack.  All mower motion commands are
translated directly on the microcontroller and forwarded to the mower's RS485
bus.  Incoming messages from the mower are published back to ROS.  The CC line
on the Type‑C connector is held low on boot to disable the mower's sensors, as
described in the ArduPilot forum thread.

Communication over RS485 uses 0x55AA header messages described in the thread:

- 2 byte header `0x55 AA`
- 1 byte message ID
- 1 byte length (always 14)
- 8 data bytes
- 2 byte CRC16 (polynomial `0x1021`)

The serial port for RS485 runs at 460800 bps.  Velocity commands from the ROS
topic `/ll/cmd_vel` are converted to wheel speed messages and sent on the RS485
bus using the 0x55AA protocol.

Build the firmware using PlatformIO.  The `micro_ros_arduino` library is used to
provide ROS 2 communication over the USB serial connection.
