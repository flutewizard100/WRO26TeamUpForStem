#include <Servo.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <CytronMotorDriver.h>



struct Watchdog {
  unsigned long last_time = 0;
  unsigned long timeout_ms = 500;

  void reset() {
    last_time = millis();
  }

  bool timed_out() {
    return (millis() - last_time) > timeout_ms;
  }
};

const int SERVO_PIN = 9; 
Servo myServo;
Watchdog motor_wd;
Watchdog servo_wd;
CytronMD motor(PWM_DIR, 2, 3);


rcl_subscription_t servo_subscriber;
std_msgs__msg__Int32 servo_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32 motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;




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

}

void motor_callback(const void * msgin) {

  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  int speed = msg->data;
  motor.setSpeed(speed);

  
}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(90); 

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));


  RCCHECK(rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Servo"
  ));

  RCCHECK(rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Motor"
  ));


  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_callback, ON_NEW_DATA));
}

void loop() {
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}

