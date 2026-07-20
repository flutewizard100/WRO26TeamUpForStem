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

const int SERVO_PIN = 9;
const int MOTOR_PIN = 12;
Servo myServo;
Servo ESC;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int32 servo_msg;

rcl_publisher_t debug_publisher;
std_msgs__msg__Int32 debug_msg;

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void servo_callback(const void * msvin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  int angle = msg->data;
  myServo.write(angle);

  debug_msg.data = angle;
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

}

void motor_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  target_speed = msg->data; 

  debug_msg.data = msg->data;
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  
}

void keyboard_callback(const void * msgin) {

  const geometry_msgs__msg__Twist *msg_in = (const geometry_msgs__msg__Twist *)msgin;
  float direction = msg_in->linear.x;
  if (direction < 0) {

    target_speed = 1350;

  } else if (direction > 0) {

    target_speed = 1590;

  } else if (direction == 0) {

    target_speed = 1500;

  }

  debug_msg.data = target_speed;
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));


}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(90); 
  
  // Safe default range limit setup matching your calibration
  ESC.attach(MOTOR_PIN, 1000, 2000);
  ESC.writeMicroseconds(1500); // Startup at safe neutral

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));

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
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "debug_speed"
  ));


  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_callback, ON_NEW_DATA));

}

void loop() {
  // Let the micro-ROS executor process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // DIRECT PASSTHROUGH TO THE ESC
  // Updates instantly based on your Python `try/finally` logic safely driving it
  ESC.writeMicroseconds(target_speed);
}
