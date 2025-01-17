#ifndef _MONICA_ROS_COMM_H_
#define _MONICA_ROS_COMM_H_

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>    // publisher, subscription, node <rcl/init.h>: allocator, support
#include <rclc/executor.h>  // executor
#include <std_msgs/msg/int32.h> // std_msgs__msg__Int32
#include <geometry_msgs/msg/twist.h>  // geometry_msgs__msg__Twist
#include <geometry_msgs/msg/quaternion.h>   // geometry_msgs__msg__Quaternion

#include "MonicaMotor.h"
#include "MonicaSong.h"
#include "MonicaDisplay.h"
#include "MonicaLed.h"
#include "MonicaMPU6050.h"


#define DOMAINID 82

typedef enum  
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
  
} AFENT_STATES;

class MonicaRosComm
{
public:
    MonicaRosComm(MonicaMotor* motor_left, 
                  MonicaMotor* motor_right,
                  MonicaSong* song,
                  MonicaDisplay* display,
                  MonicaLed* led,
                  MonicaMPU6050* mpu);
    ~MonicaRosComm();

public:
    void Initialize();
    void check_agent_state();
    void ready_to_publish_message();
    void publish_messages();


public:
    void set_motor_left(MonicaMotor* motor_left) { motor_left_ = motor_left; }
    void set_motor_right(MonicaMotor* motor_right) { motor_right_ = motor_right; }

    void set_song(MonicaSong* song) { song_=song; }
    void set_display(MonicaDisplay* display) { display_ = display; }
    void set_led(MonicaLed* led) { led_ = led; }

    void set_mpu(MonicaMPU6050* mpu) { mpu_ = mpu; }

    float get_last_cmd_vel_received_time() { return lastCmdVelReceived; }

private:
    static void cmd_vel_callback(const void *msgin);
    static void subled_callback(const void *msgin);
    static void subsong_callback(const void *msgin);
    static void sublcd_callback(const void *msgin);

private:
    //===============================================
    // Motor ========================================
    //===============================================
    static MonicaMotor* motor_left_;
    static MonicaMotor* motor_right_;
    static float lastCmdVelReceived; // Record the time that the last velocity command was received

    rcl_publisher_t left_pub, right_pub;
    std_msgs__msg__Int32 msg_left_tick_count, msg_right_tick_count;

    rcl_subscription_t cmd_vel_sub;
    geometry_msgs__msg__Twist cmd_vel;

    //===============================================


    //===============================================
    // Led, Buzzer, LCD =============================
    //===============================================
    static MonicaSong* song_;
    static MonicaDisplay* display_;
    static MonicaLed* led_;
    
    rcl_subscription_t ledSub, songSub, lcdSub;
    std_msgs__msg__Int32 ledMsg, songMsg, lcdMsg;
    //===============================================


    //===============================================
    // MPU6050 ======================================
    //===============================================
    static MonicaMPU6050* mpu_;

    rcl_publisher_t quat_pub;
    geometry_msgs__msg__Quaternion quat_msg;
    //===============================================


    //===============================================
    // For ROS2 =====================================
    //===============================================
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    AFENT_STATES state;
    //===============================================
};
#endif /*_MONICA_ROS_COMM_H_*/
