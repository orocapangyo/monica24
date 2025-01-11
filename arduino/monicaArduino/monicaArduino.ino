#include <micro_ros_arduino.h>
#include <Arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <PID_v1.h>
#include <geometry_msgs/msg/quaternion.h>

#include "songlcdled.h"

#include "MonicaMotor.h"
#include "MotorModule.h"

#define DOMAINID 82

#define MOTOR_60RPM 1
#define MOTOR_178RPM 2
#define MOTOR_TYPE MOTOR_60RPM

#define PRINT_VEL 0
#define PRINT_PIDERR 0
#define PRINT_AVGPWM 0
#define PRINT_INSPWM 0
#define PRINT_QUAT 0



rcl_publisher_t left_pub, right_pub;
rcl_subscription_t cmd_vel_sub;
std_msgs__msg__Int32 msg_right_tick_count, msg_left_tick_count;
geometry_msgs__msg__Twist cmd_vel;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define RXD2 16
#define TXD2 17

#define DEBUG 1
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


// Encoder output to Arduino Interrupt pin. Tracks the tick count
#if MOTOR_TYPE == MOTOR_60RPM
#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34
#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35
#else
#define ENC_IN_LEFT_A 39
#define ENC_IN_RIGHT_A 35
#define ENC_IN_LEFT_B 36
#define ENC_IN_RIGHT_B 34
#endif
// Motor A, B control
#define AIN1 26
#define AIN2 25
#define BIN1 27
#define BIN2 14

// Motor speed control via PWM
#define ENA 32
#define ENB 33
#define ENA_CH 0
#define ENB_CH 1

#define STBY 4




MonicaMotor motor_left(ENC_IN_LEFT_A, ENC_IN_LEFT_B, AIN1, AIN2, STBY, ENA, ENA_CH);
MonicaMotor motor_right(ENC_IN_RIGHT_A, ENC_IN_RIGHT_B, BIN1, BIN2, STBY, ENB, ENB_CH);

MotorModule motor_module(&motor_left, &motor_right);



#define INTERVAL 50  //50ms, so 20Hz

long previousMillis = 0;
long currentMillis = 0;
// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      char buffer[40]; \
      sprintf(buffer, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      DEBUG_PRINTLN(buffer); \
    } \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)




// Take the velocity command as input and calculate the PWM values.
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *cmdVel = (const geometry_msgs__msg__Twist *)msgin;

  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000.0);

  float vLeft = 0.0;
  float vRight = 0.0;

  int pwmLeftReq = 0;
  int pwmRightReq = 0;

  vLeft = cmdVel->linear.x - cmdVel->angular.z * WHEEL_BASE / 2.0;
  vRight = cmdVel->linear.x + cmdVel->angular.z * WHEEL_BASE / 2.0;

  if (vLeft >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = int(K_P * vLeft + K_b + K_Lbias);
  } else {
    pwmLeftReq = int(K_P * vLeft - K_b - K_Lbias);
  }
  if (vRight >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmRightReq = K_P * vRight + K_b;
  } else {
    pwmRightReq = K_P * vRight - K_b;
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }

  motor_left.set_req_vel(vLeft);
  motor_right.set_req_vel(vRight);

  motor_left.set_pwm_req(pwmLeftReq);
  motor_right.set_pwm_req(pwmRightReq);

  //reset and start again PID controller
  motor_left.restart_pid();
  motor_right.restart_pid();
}

void setup() {
  int i;
#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("Enc/Motor/MPU6050/Other Starts");

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_F, OUTPUT);
  RGB(ALL_OFF);  // RGB LED all off


  motor_left.initialize();
  motor_right.initialize();

  motor_module.initialize();

  
  ledcSetup(BUZZER, 1000, 8);  //enB, channel: 1, 1000Hz, 8bits = 256(0 ~ 255)
  ledcAttachPin(BUZZER, BUZZER_CH);
  ledcWrite(BUZZER, 0);

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  beginLcd();

  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");
  set_microros_transports();

  delay(2000);

  //wait agent comes up
  do {
    EXECUTE_EVERY_N_MS(300, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(300);

    if ((i++ % 32) == 0) DEBUG_PRINT("\n");
    else DEBUG_PRINT(".");
    if (state == AGENT_AVAILABLE)
      break;
  } while (1);

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, DOMAINID);
  // create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  DEBUG_PRINTLN("rclc_support_init done");

  RCCHECK(rclc_node_init_default(&node, "uros_arduino_node", "", &support));
  DEBUG_PRINTLN("rclc_node_init done");

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &right_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_ticks"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &left_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_ticks"));
;

  // create cmd_vel subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, cmd_vel_callback, ON_NEW_DATA));

  DEBUG_PRINTLN("ROS established");
  DEBUG_PRINTLN("Done setup");
}

void loop() {
  switch (state) {
    case AGENT_AVAILABLE:
      //if setup done, always here. then go to next step
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      //if ping doesn't work, then reset board(=wait agent comes up)
      ESP.restart();
      break;
    default:
      break;
  }

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    // Calculate the velocity of the right and left wheels
    motor_left.calc_vel();
    motor_right.calc_vel();

    msg_left_tick_count.data = motor_left.get_wheel_tick_count();
    msg_right_tick_count.data = motor_right.get_wheel_tick_count();

    RCSOFTCHECK(rcl_publish(&left_pub, &msg_left_tick_count, NULL));
    RCSOFTCHECK(rcl_publish(&right_pub, &msg_right_tick_count, NULL));

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    motor_left.set_pwm_req(0);
    motor_right.set_pwm_req(0);
  }

  motor_left.set_pwm_values();
  motor_right.set_pwm_values();
}


