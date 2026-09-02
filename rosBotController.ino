#include <Servo.h>

#include <PololuMaestro.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
const int SERVO_PIN = 9;
const int MOTOR_PIN = 12;
char debug_buffer[64];
Servo myServo;
Servo ESC;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int32 servo_msg;

rcl_publisher_t debug_publisher;
std_msgs__msg__String debug_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32 motor_msg;

rcl_subscription_t direction_subscriber;
geometry_msgs__msg__Twist direction_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Direct variable to hold incoming speed targets from ROS
int target_speed = 1500;
int servo_angle = 80;

// Command-staleness failsafe: if no message from the host arrives within
// this many milliseconds, force motor to neutral and steering to center.
// Any inbound callback resets last_cmd_ms.
#define MOTOR_TIMEOUT_MS 1000
#define SERVO_CENTER 80
#define SERVO_MIN 40
#define SERVO_MAX 150
#define ESC_MIN 1300
#define ESC_MAX 1700
// cmd_vel scaling. Flip TELEOP_TURN_SCALE sign if steering is mirrored.
#define TELEOP_SPEED_SCALE 200
#define TELEOP_TURN_SCALE 40
unsigned long last_cmd_ms = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void servo_callback(const void *msvin)
{
  // servo_angle is an ABSOLUTE steering angle in degrees. Values outside
  // [SERVO_MIN, SERVO_MAX] are clamped. For teleop-style Twist input the
  // firmware maps cmd_vel directly in keyboard_callback; this path is for
  // programmatic Python callers (Servo.write, mission code).
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msvin;
  last_cmd_ms = millis();

  int ang = msg->data;
  if (ang < SERVO_MIN) ang = SERVO_MIN;
  if (ang > SERVO_MAX) ang = SERVO_MAX;
  servo_angle = ang;

  myServo.write(servo_angle);

  snprintf(debug_buffer, sizeof(debug_buffer),
         "Speed: %d  Angle: %d",
         target_speed,
         servo_angle);

  rosidl_runtime_c__String__assign(&debug_msg.data, debug_buffer);
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}


void motor_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  last_cmd_ms = millis();

  int command = msg->data;

  target_speed = command;

  snprintf(debug_buffer, sizeof(debug_buffer),
         "Speed: %d  Angle: %d",
         target_speed,
         servo_angle);

  rosidl_runtime_c__String__assign(&debug_msg.data, debug_buffer);
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

void keyboard_callback(const void * msgin) {

  const geometry_msgs__msg__Twist *msg_in = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();
  float direction = msg_in->linear.x;
  float turn = msg_in->angular.z;

  int spd = 1500 + (int)(direction * TELEOP_SPEED_SCALE);
  if (spd < ESC_MIN) spd = ESC_MIN;
  if (spd > ESC_MAX) spd = ESC_MAX;
  target_speed = spd;

  int ang = SERVO_CENTER - (int)(turn * TELEOP_TURN_SCALE);
  if (ang < SERVO_MIN) ang = SERVO_MIN;
  if (ang > SERVO_MAX) ang = SERVO_MAX;
  servo_angle = ang;

  snprintf(debug_buffer, sizeof(debug_buffer),
         "Speed: %d  Angle: %d",
         target_speed,
         servo_angle);

  myServo.write(servo_angle);

  rosidl_runtime_c__String__assign(&debug_msg.data, debug_buffer);
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));


}

bool wait_for_agent()
{
  const int timeout_ms = 1000;
  const int attempts = 1;

  while (rmw_uros_ping_agent(timeout_ms, attempts) != RMW_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }

  return true;
}

void setup() {


  pinMode(LED_BUILTIN, OUTPUT);

  // Bring actuators to a known-safe state BEFORE blocking on the micro-ROS agent,
  // so a cold boot without a host still leaves the servo held at center and the
  // ESC seeing a neutral pulse (also serves as the ESC arm signal).
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_CENTER);

  ESC.attach(MOTOR_PIN, 1000, 2000);
  ESC.writeMicroseconds(1500);

  set_microros_transports();

  wait_for_agent();


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));
  std_msgs__msg__String__init(&debug_msg);
  RCCHECK(rclc_subscription_init_default(
    &direction_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));


  RCCHECK(rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "servo_angle"
  ));

  RCCHECK(rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_speed"
  ));


  RCCHECK(rclc_publisher_init_default(
  &debug_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
  "debug_speed"
  ));


  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &direction_subscriber, &direction_msg, &keyboard_callback, ON_NEW_DATA));

}

void loop() {
  // Let the micro-ROS executor process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Command-staleness failsafe: if the host has gone quiet, neutralize.
  // Auto-resumes as soon as any callback updates last_cmd_ms.
  if (millis() - last_cmd_ms > MOTOR_TIMEOUT_MS) {
    target_speed = 1500;
    servo_angle = SERVO_CENTER;
    myServo.write(SERVO_CENTER);
  }

  // DIRECT PASSTHROUGH TO THE ESC
  // Updates instantly based on your Python `try/finally` logic safely driving it
  ESC.writeMicroseconds(target_speed);
}
