#include <Arduino.h>
#include <stdio.h>

#include <rclc/rclc.h> //rclc/init.h: rclc_support_init_with_options()

#include "MonicaRosComm.h"


#define DEBUG 1
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

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


MonicaMotor* MonicaRosComm::motor_left_ = nullptr;
MonicaMotor* MonicaRosComm::motor_right_ = nullptr;
float MonicaRosComm::lastCmdVelReceived = 0.0;

MonicaSong* MonicaRosComm::song_ = nullptr;
MonicaDisplay* MonicaRosComm::display_ = nullptr;
MonicaLed* MonicaRosComm::led_ = nullptr;

MonicaMPU6050* MonicaRosComm::mpu_ = nullptr;

MonicaRosComm::MonicaRosComm(MonicaMotor* motor_left, 
                             MonicaMotor* motor_right,
                             MonicaSong* song,
                             MonicaDisplay* display,
                             MonicaLed* led,
                             MonicaMPU6050* mpu)
{
  set_motor_left(motor_left);
  set_motor_right(motor_right);
  set_song(song);
  set_display(display);
  set_led(led);
  set_mpu(mpu);

  set_used_wifi(false);
  set_domain_id(DEFAULT_DOMAINID);
  memset(ssid_, 0, 64);
  memset(password_, 0, 32);
  memset(agent_ip_, 0, 32);
  set_port(DEFAULT_PORT);

  is_first_initialized_ = false;
}

MonicaRosComm::~MonicaRosComm()
{

}

void MonicaRosComm::Initialize()
{
  // ROS Setup
  DEBUG_PRINTLN("ROS Starts");

  if (is_first_initialized_)
    return;

  is_first_initialized_ = true;

  int i;
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
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id_));
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

  // create cmd_vel subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));




  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &quat_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "quaternion"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &ledSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ledSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &songSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "songSub"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &lcdSub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lcdSub"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, MonicaRosComm::cmd_vel_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(&executor, &ledSub, &ledMsg, MonicaRosComm::subled_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &songSub, &songMsg, MonicaRosComm::subsong_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lcdSub, &lcdMsg, MonicaRosComm::sublcd_callback, ON_NEW_DATA));

  DEBUG_PRINTLN("ROS established");
}



void MonicaRosComm::check_agent_state()
{
  switch (state) 
  {
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
      //ESP.restart();
      state = AGENT_CONNECTED;
      break;
    default:
      break;
  }
}


void MonicaRosComm::ready_to_publish_message()
{
  if (motor_left_ != nullptr)
  {
    msg_left_tick_count.data = motor_left_->get_wheel_tick_count();
  }
  if (motor_right_ != nullptr)
  {
    msg_right_tick_count.data = motor_right_->get_wheel_tick_count();
  }

  if (mpu_ != nullptr)
  {
    quat_msg.w = mpu_->get_quaternion()->w;
    quat_msg.x = mpu_->get_quaternion()->x;
    quat_msg.y = mpu_->get_quaternion()->y;
    quat_msg.z = mpu_->get_quaternion()->z;
  }
}


void MonicaRosComm::publish_messages()
{
  RCSOFTCHECK(rcl_publish(&left_pub, &msg_left_tick_count, NULL));
  RCSOFTCHECK(rcl_publish(&right_pub, &msg_right_tick_count, NULL));
  RCSOFTCHECK(rcl_publish(&quat_pub, &quat_msg, NULL));
}



void MonicaRosComm::use_wifi_transform()
{
  //Serial.println(ssid_);
  //Serial.println(password_);
  //Serial.println(agent_ip_);
  //Serial.println(port_);
  set_microros_wifi_transports(ssid_, password_, agent_ip_, port_);
}

void MonicaRosComm::use_serial_transform()
{
    set_microros_transports();
}


// Take the velocity command as input and calculate the PWM values.
void MonicaRosComm::cmd_vel_callback(const void *msgin) 
{
  if (motor_left_ == nullptr && motor_right_ == nullptr)
  {
    return;
  }

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

  motor_left_->set_req_vel(vLeft);
  motor_right_->set_req_vel(vRight);

  motor_left_->set_pwm_req(pwmLeftReq);
  motor_right_->set_pwm_req(pwmRightReq);

  //reset and start again PID controller
  motor_left_->restart_pid();
  motor_right_->restart_pid();
}



void MonicaRosComm::subled_callback(const void *msgin) 
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  //RGB(int(msg->data));
  if (led_ != nullptr)
  {
    led_->RGB(int(msg->data));
  }
}


void MonicaRosComm::subsong_callback(const void *msgin) 
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  
  //playsong(int(msg->data));
  if (song_ != nullptr)
  {
    song_->stop();
    song_->select(int(msg->data));
  }
}


void MonicaRosComm::sublcd_callback(const void *msgin) 
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  //showAnimation(int(msg->data));
  if (display_ != nullptr)
  {
    display_->clear();
    display_->select(int(msg->data));
  }
}