#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

class ESP32SerialBridge:
    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud = rospy.get_param('~baud', 115200)
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {port}: {e}")
            self.ser = None

        self.cmd_sub = rospy.Subscriber('mower_control', String, self.send_command)
        self.status_pub = rospy.Publisher('mower_status', String, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.read_serial)

    def send_command(self, msg):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((msg.data + '\n').encode('utf-8'))
            except serial.SerialException as e:
                rospy.logerr(f"Serial write failed: {e}")

    def read_serial(self, event):
        if self.ser and self.ser.is_open and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.status_pub.publish(line)
            except serial.SerialException as e:
                rospy.logerr(f"Serial read failed: {e}")


def main():
    rospy.init_node('esp32_serial_bridge')
    ESP32SerialBridge()
    rospy.loginfo('ESP32 serial bridge started')
    rospy.spin()

if __name__ == '__main__':
    main()
