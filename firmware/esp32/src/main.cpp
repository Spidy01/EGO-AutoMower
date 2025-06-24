#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/bool.h>

// RS485 serial port at 460800 baud
HardwareSerial RS485(1);

// Pin definitions
const int RS485_TX_PIN = 17;
const int RS485_RX_PIN = 18;
const int RS485_DE_PIN = 16;  // Driver enable
const int CC_PIN = 4;         // Pulls Type-C CC line low
const int BLADE_PIN = 5;      // Controls blade motors
const int SEAT_PIN = 6;       // Simulates seat switch
const int ESTOP_PIN = 7;      // Emergency stop input (NC = healthy)

// micro-ROS objects
rcl_publisher_t rx_pub;
rcl_subscription_t tx_sub;
rcl_subscription_t blade_sub;
rcl_subscription_t seat_sub;
rcl_subscription_t bus_enable_sub;
rcl_publisher_t estop_pub;
rcl_timer_t estop_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static bool bus_enabled = false;

void setup() {
  pinMode(CC_PIN, OUTPUT);
  digitalWrite(CC_PIN, HIGH);  // keep sensors enabled until autonomous mode

  pinMode(BLADE_PIN, OUTPUT);
  digitalWrite(BLADE_PIN, LOW);

  pinMode(SEAT_PIN, OUTPUT);
  digitalWrite(SEAT_PIN, LOW);

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial.begin(115200);
  RS485.begin(460800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "ego_mower_bridge", "", &support);
  rclc_publisher_init_default(&rx_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                              "rs485_rx");
  rclc_subscription_init_default(&tx_sub, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                                 "rs485_tx");
  rclc_subscription_init_default(&blade_sub, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                 "blade_enable");
  rclc_subscription_init_default(&seat_sub, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                 "motion_enable");
  rclc_subscription_init_default(&bus_enable_sub, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                 "bus_enable");
  rclc_publisher_init_default(&estop_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                              "estop_state");

  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_subscription(&executor, &tx_sub, NULL, [](const void* msg){
      if(!bus_enabled) return;
      const std_msgs__msg__UInt8MultiArray* m = (const std_msgs__msg__UInt8MultiArray*)msg;
      digitalWrite(RS485_DE_PIN, HIGH);
      for(size_t i=0;i<m->data.size;i++) {
          RS485.write(m->data.data[i]);
      }
      RS485.flush();
      digitalWrite(RS485_DE_PIN, LOW);
  }, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &blade_sub, NULL, [](const void* msg){
      const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg;
      digitalWrite(BLADE_PIN, m->data ? HIGH : LOW);
  }, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &seat_sub, NULL, [](const void* msg){
      const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg;
      digitalWrite(SEAT_PIN, m->data ? HIGH : LOW);
  }, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &bus_enable_sub, NULL, [](const void* msg){
      const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msg;
      bus_enabled = m->data;
      digitalWrite(CC_PIN, bus_enabled ? LOW : HIGH);
  }, ON_NEW_DATA);
  rclc_timer_init_default(&estop_timer, &support, RCL_MS_TO_NS(100), [](rcl_timer_t *timer, int64_t last_call){
      (void)timer; (void)last_call;
      std_msgs__msg__Bool out_msg;
      out_msg.data = digitalRead(ESTOP_PIN) == HIGH;
      rcl_publish(&estop_pub, &out_msg, NULL);
  });
  rclc_executor_add_timer(&executor, &estop_timer);
}

void loop() {
  static uint8_t buffer[14];
  static size_t index = 0;
  while (RS485.available()) {
    buffer[index++] = RS485.read();
    if(index >= sizeof(buffer)) {
      std_msgs__msg__UInt8MultiArray out_msg;
      out_msg.data.data = buffer;
      out_msg.data.size = index;
      out_msg.data.capacity = sizeof(buffer);
      rcl_publish(&rx_pub, &out_msg, NULL);
      index = 0;
    }
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
