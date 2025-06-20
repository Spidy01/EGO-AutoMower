#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// RS485 serial port at 460800 baud
HardwareSerial RS485(1);  // use UART1

// Pin definitions
const int RS485_TX_PIN = 17;
const int RS485_RX_PIN = 18;
const int RS485_DE_PIN = 16;  // Driver enable
const int CC_PIN = 4;         // Pulls Type-C CC line low

// Wheel distance [m] used for diff drive. Adjust to your mower.
static const float WHEEL_BASE = 0.60f;  // placeholder

// micro-ROS objects
rcl_publisher_t rx_pub;
rcl_subscription_t cmd_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Helper to compute CRC16 (polynomial 0x1021)
uint16_t crc16_update(uint16_t crc, uint8_t a) {
  crc ^= ((uint16_t)a << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

void send_message(uint8_t id, const uint8_t* data) {
  uint8_t frame[14];
  frame[0] = 0x55;
  frame[1] = 0xAA;
  frame[2] = id;
  frame[3] = 14;  // length
  memcpy(&frame[4], data, 8);
  uint16_t crc = 0;
  for (int i = 0; i < 12; ++i) {
    crc = crc16_update(crc, frame[i]);
  }
  frame[12] = crc >> 8;
  frame[13] = crc & 0xFF;

  digitalWrite(RS485_DE_PIN, HIGH);  // enable driver
  RS485.write(frame, 14);
  RS485.flush();
  digitalWrite(RS485_DE_PIN, LOW);  // back to receive
}

void cmd_vel_callback(const void* msg_in) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  float left = msg->linear.x - 0.5f * WHEEL_BASE * msg->angular.z;
  float right = msg->linear.x + 0.5f * WHEEL_BASE * msg->angular.z;

  int16_t left_mm_s = (int16_t)(left * 1000.0f);   // scale to mm/s
  int16_t right_mm_s = (int16_t)(right * 1000.0f); // scale to mm/s

  uint8_t data[8] = {0};
  data[0] = left_mm_s >> 8;
  data[1] = left_mm_s & 0xFF;
  data[2] = right_mm_s >> 8;
  data[3] = right_mm_s & 0xFF;
  // remaining bytes unused
  send_message(0x10, data);  // 0x10: assumed ID for velocity command
}

void setup() {
  pinMode(CC_PIN, OUTPUT);
  digitalWrite(CC_PIN, LOW);  // disable sensors by pulling CC low

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.begin(115200);
  RS485.begin(460800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "ego_mower_bridge", "", &support);
  rclc_publisher_init_default(&rx_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                              "ll/rs485_raw");
  rclc_subscription_init_default(&cmd_sub, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                 "ll/cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &geometry_msgs__msg__Twist__create(),
                                &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  if (RS485.available()) {
    String line;
    while (RS485.available()) {
      int b = RS485.read();
      if (b == '\n') break;
      if (b >= 0) line += (char)b;
    }
    if (line.length() > 0) {
      std_msgs__msg__String msg;
      msg.data.data = (char*)line.c_str();
      msg.data.size = line.length();
      msg.data.capacity = line.length() + 1;
      rcl_publish(&rx_pub, &msg, NULL);
    }
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

