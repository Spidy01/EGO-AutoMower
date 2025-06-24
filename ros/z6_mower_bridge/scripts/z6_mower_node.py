#!/usr/bin/env python3
"""Z6 mower translation node.

This node translates between open_mower_ros topics/services and the
binary RS485 protocol used by the Ego Z6 mower. It forwards blade,
motion and bus enable commands to the ESP32 and publishes the emergency
stop state. The actual mower protocol is not fully known so many
translations are left as TODO.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray, Bool
from std_srvs.srv import SetBool, SetBoolResponse

RS485_TX_TOPIC = 'rs485_tx'
RS485_RX_TOPIC = 'rs485_rx'

CMD_VEL_TOPIC = 'll/cmd_vel'
MOWER_ENABLE_SERVICE = 'll/_service/mow_enabled'
COLLISION_TOPIC = 'll/collision_detected'
BLADE_SERVICE = 'll/_service/blade_switch'
MOTION_SERVICE = 'll/_service/motion_enable'
BLADE_TOPIC = 'blade_enable'
MOTION_TOPIC = 'motion_enable'
ESTOP_TOPIC = 'estop_state'
BUS_ENABLE_TOPIC = 'bus_enable'

# Message IDs (guessed from forum information)
MSG_VELOCITY = 0x10
MSG_MOWER_ENABLE = 0x20
MSG_COLLISION = 0x30

HEADER = [0x55, 0xAA]
FRAME_LEN = 14


def build_frame(msg_id, payload):
    """Build RS485 frame with CRC16. Payload must be 8 bytes."""
    frame = bytearray(HEADER)
    frame.append(msg_id)
    frame.append(FRAME_LEN)
    data = bytearray(8)
    data[:len(payload)] = payload
    frame.extend(data)
    crc = 0
    for b in frame:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    frame.append((crc >> 8) & 0xFF)
    frame.append(crc & 0xFF)
    return frame


def twist_to_frame(msg):
    left = msg.linear.x - 0.3 * msg.angular.z  # wheelbase approx
    right = msg.linear.x + 0.3 * msg.angular.z
    left_mm_s = int(left * 1000)
    right_mm_s = int(right * 1000)
    payload = bytearray(8)
    payload[0] = (left_mm_s >> 8) & 0xFF
    payload[1] = left_mm_s & 0xFF
    payload[2] = (right_mm_s >> 8) & 0xFF
    payload[3] = right_mm_s & 0xFF
    return build_frame(MSG_VELOCITY, payload)


def mower_enable_frame(enabled):
    payload = bytearray(8)
    payload[0] = 1 if enabled else 0
    return build_frame(MSG_MOWER_ENABLE, payload)


class Z6Bridge:
    def __init__(self):
        self.tx_pub = rospy.Publisher(RS485_TX_TOPIC, UInt8MultiArray, queue_size=10)
        self.rx_sub = rospy.Subscriber(RS485_RX_TOPIC, UInt8MultiArray, self.rx_cb)
        self.cmd_sub = rospy.Subscriber(CMD_VEL_TOPIC, Twist, self.cmd_cb)
        self.collision_pub = rospy.Publisher(COLLISION_TOPIC, Bool, queue_size=10)
        self.srv = rospy.Service(MOWER_ENABLE_SERVICE, SetBool, self.handle_enable)
        self.blade_srv = rospy.Service(BLADE_SERVICE, SetBool, self.handle_blade)
        self.motion_srv = rospy.Service(MOTION_SERVICE, SetBool, self.handle_motion)
        self.blade_pub = rospy.Publisher(BLADE_TOPIC, Bool, queue_size=1)
        self.motion_pub = rospy.Publisher(MOTION_TOPIC, Bool, queue_size=1)
        self.bus_pub = rospy.Publisher(BUS_ENABLE_TOPIC, Bool, queue_size=1)
        self.estop_sub = rospy.Subscriber(ESTOP_TOPIC, Bool, self.estop_cb)
        self.estop_state = True

    def cmd_cb(self, msg):
        frame = twist_to_frame(msg)
        self.publish_frame(frame)

    def handle_enable(self, req):
        frame = mower_enable_frame(req.data)
        self.publish_frame(frame)
        self.bus_pub.publish(Bool(data=req.data))
        return SetBoolResponse(success=True, message='')

    def handle_blade(self, req):
        self.blade_pub.publish(Bool(data=req.data))
        return SetBoolResponse(success=True, message='')

    def handle_motion(self, req):
        self.motion_pub.publish(Bool(data=req.data))
        return SetBoolResponse(success=True, message='')

    def estop_cb(self, msg):
        self.estop_state = msg.data

    def publish_frame(self, frame):
        arr = UInt8MultiArray(data=list(frame))
        self.tx_pub.publish(arr)

    def rx_cb(self, msg):
        if len(msg.data) < 5:
            return
        msg_id = msg.data[2]
        if msg_id == MSG_COLLISION:
            collision = msg.data[4] != 0
            self.collision_pub.publish(Bool(data=collision))
        else:
            # TODO: handle additional incoming message types
            pass


if __name__ == '__main__':
    rospy.init_node('z6_mower_bridge')
    Z6Bridge()
    rospy.loginfo('Z6 mower bridge started')
    rospy.spin()
