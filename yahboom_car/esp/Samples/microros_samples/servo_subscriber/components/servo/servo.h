#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdbool.h"
#include "stdint.h"

#define SERVO_GPIO_S1                  8       // GPIO connects to the PWM signal line
#define SERVO_GPIO_S2                  21      // GPIO connects to the PWM signal line
#define SERVO_TIMER_GROUP_ID           1       // The pwm timer group, esp32s3 only can slect 0 or 1
#define SERVO_TIMEBASE_RESOLUTION_HZ   1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD          20000   // 20000 ticks, 20ms

#define SERVO_MIN_PULSEWIDTH_US        500     // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US        2500    // Maximum pulse width in microsecond
#define SERVO_MIN_HD_ANGLE             -90     // Minimum Angle of Servo
#define SERVO_MAX_HD_ANGLE             90      // Maximum Angle of Servo


// 配置舵机的最大值和最小值，舵机自身限制范围为[-90, 90]，请在此范围内调整各舵机的控制范围。
// Configure the maximum and minimum values of the steering gear.
// The limiting range of the steering gear itself is [-90, 90].
// Please adjust the control range of each steering gear within this range.
#define SERVO_S1_DEF_ANGLE (0)   // S1默认角度
#define SERVO_S1_MIN_ANGLE (-90) // 限制S1最小角度
#define SERVO_S1_MAX_ANGLE (90)  // 限制S1最大角度

#define SERVO_S2_DEF_ANGLE (-60) // S2默认角度
#define SERVO_S2_MIN_ANGLE (-90) // 限制S2最小角度
#define SERVO_S2_MAX_ANGLE (20)  // 限制S2最大角度

typedef enum _servo_id
{
    SERVO_ID_S1 = 0,
    SERVO_ID_S2 = 1,

    SERVO_ID_MAX
} servo_id_t;

void Servo_Init(void);
void Servo_Set_Angle(servo_id_t servo, int8_t angle);

#ifdef __cplusplus
}
#endif
