#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h> // Required for text messages
#include <Servo.h>

rcl_node_t node;
rcl_publisher_t publisher;
rcl_publisher_t log_publisher;   // Custom log channel
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Int32 msg;
std_msgs__msg__String log_msg;   // Custom text log message
Servo myServo;

#define LED_PIN 13 
const int SERVO_PIN = 3;

// Non-blocking timing and servo tracking variables
unsigned long last_servo_time = 0;
unsigned long last_publish_time = 0;
int servo_angle = 0;
int sweep_direction = 1;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(servo_angle);

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // LOOP UNTIL CONNECTED TO AGENT
  while(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(500);
  }

  // Initialize node
  while(rclc_node_init_default(&node, "teensy_test_node", "", &support) != RCL_RET_OK){
    delay(500);
  }

  // Initialize integer data publisher
  while(rclc_publisher_init_default(&publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teensy_counter") != RCL_RET_OK){
    delay(500);
  }

  // Initialize functioning text log publisher
  while(rclc_publisher_init_default(&log_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "teensy_log_stream") != RCL_RET_OK){
    delay(500);
  }

  digitalWrite(LED_PIN, HIGH); 
  msg.data = 0;
}

void loop()
{
    unsigned long current_time = millis();
  
  // 1. NON-BLOCKING PUBLISH (Runs every 1000ms)
  if (current_time - last_publish_time >= 1000) {
    last_publish_time = current_time;

    // Publish your counter data
    if (RCL_RET_OK == rcl_publish(&publisher, &msg, NULL)) {
      
      // FIX: Changed 'char log_buffer' to an explicit array size 'char log_buffer[64]'
      char log_buffer[64];
      snprintf(log_buffer, sizeof(log_buffer), "Published counter value: %d", (int)msg.data);
      
      log_msg.data.data = log_buffer;
      log_msg.data.size = strlen(log_buffer);
      log_msg.data.capacity = sizeof(log_buffer);
      
      rcl_publish(&log_publisher, &log_msg, NULL);
      
      msg.data++;
    }
  }

    // 2. NON-BLOCKING SERVO SWEEP (Steps 1 degree every 15ms)
    if (current_time - last_servo_time >= 15) {
      last_servo_time = current_time;

      servo_angle += sweep_direction;

      if (servo_angle >= 180) {
        servo_angle = 180;
        sweep_direction = -1;
      } else if (servo_angle <= 0) {
        servo_angle = 0;
        sweep_direction = 1;
      }

      myServo.write(servo_angle);
    }

}
