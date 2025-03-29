#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

// 电机引脚引脚定义 Motor pin pin definition
#define PWM_GPIO_M1A              4
#define PWM_GPIO_M1B              5

#define PWM_GPIO_M2A              15
#define PWM_GPIO_M2B              16

#define PWM_GPIO_M3A              9
#define PWM_GPIO_M3B              10

#define PWM_GPIO_M4A              13
#define PWM_GPIO_M4B              14



// PWM电机时钟频率, 10MHz, 1 tick = 0.1us 
// PWM motor clock frequency, 10MHz, 1 tick = 0.1us
#define PWM_MOTOR_TIMER_RESOLUTION_HZ    10000000

// PWM电机控制频率, 25KHz 
// PWM motor control frequency, 25KHz
#define PWM_MOTOR_FREQ_HZ                25000

// PWM理论最大值(400) 
// PWM Theoretical maximum (400)
#define PWM_MOTOR_DUTY_TICK_MAX          (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)

// 电机死区过滤 
// Motor dead zone filtering
#define PWM_MOTOR_DEAD_ZONE              (200)

// 电机PWM输入最大值 
// Maximum motor PWM input value
#define PWM_MOTOR_MAX_VALUE              (PWM_MOTOR_DUTY_TICK_MAX-PWM_MOTOR_DEAD_ZONE)

// 电机定时器组ID号 
// Motor timer group ID
#define PWM_MOTOR_TIMER_GROUP_ID_M1      (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M2      (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M3      (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M4      (1)


// 电机ID编号 
// Motor ID number
typedef enum _motor_id {
    MOTOR_ID_ALL = 0,
    MOTOR_ID_M1 = 1,
    MOTOR_ID_M2 = 2,
    MOTOR_ID_M3 = 3,
    MOTOR_ID_M4 = 4
} motor_id_t;

// 电机停止模式 
// Motor stop mode
typedef enum _stop_mode{
    STOP_COAST = 0,
    STOP_BRAKE = 1
} stop_mode_t;


void PwmMotor_Init(void);
void PwmMotor_Set_Speed_All(int speed_1, int speed_2, int speed_3, int speed_4);
void PwmMotor_Set_Speed(motor_id_t motor_id, int speed);
void PwmMotor_Stop(motor_id_t motor_id, bool brake);


#ifdef __cplusplus
}
#endif
