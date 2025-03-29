#include "pwm_motor.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "bdc_motor.h"


static const char *TAG = "PWM_MOTOR";


bdc_motor_handle_t motor_m1 = NULL;
bdc_motor_handle_t motor_m2 = NULL;
bdc_motor_handle_t motor_m3 = NULL;
bdc_motor_handle_t motor_m4 = NULL;

static bool stop_brake = false;

// 电机死区过滤
// Motor dead zone filtering
static int PwmMotor_Ignore_Dead_Zone(int speed)
{
    if (speed > 0) return speed + PWM_MOTOR_DEAD_ZONE;
    if (speed < 0) return speed - PWM_MOTOR_DEAD_ZONE;
    return 0;
}

// 限制输入最大值和最小值。
// Limits the maximum and minimum input values.
static int PwmMotor_Limit_Speed(int speed)
{
    if (speed > PWM_MOTOR_DUTY_TICK_MAX) return PWM_MOTOR_DUTY_TICK_MAX;
    if (speed < -PWM_MOTOR_DUTY_TICK_MAX) return -PWM_MOTOR_DUTY_TICK_MAX;
    return speed;
}

// 初始化电机1，绑定GPIO和配置定时器
// Initialize motor 1, bind GPIO and configure timer
static void PwmMotor_Init_M1(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M1B,
        .pwmb_gpio_num = PWM_GPIO_M1A,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M1,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m1 = motor;
}

// 初始化电机2，绑定GPIO和配置定时器
// Initialize motor 2, bind GPIO and configure timer
static void PwmMotor_Init_M2(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M2B,
        .pwmb_gpio_num = PWM_GPIO_M2A,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M2,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m2 = motor;
}

// 初始化电机3，绑定GPIO和配置定时器
// Initialize motor 3, bind GPIO and configure timer
static void PwmMotor_Init_M3(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M3A,
        .pwmb_gpio_num = PWM_GPIO_M3B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M3,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m3 = motor;
}

// 初始化电机4，绑定GPIO和配置定时器
// Initialize motor 4, bind GPIO and configure timer
static void PwmMotor_Init_M4(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M4A,
        .pwmb_gpio_num = PWM_GPIO_M4B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M4,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m4 = motor;
}

// 控制电机1速度，speed的输入范围：±PWM_MOTOR_MAX_VALUE
// Control motor 1 speed, speed input range: ±PWM_MOTOR_MAX_VALUE
static void PwmMotor_Set_Speed_M1(int speed)
{
    speed = PwmMotor_Ignore_Dead_Zone(speed);
    speed = PwmMotor_Limit_Speed(speed);

    if (speed > 0) // forward
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m1));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m1, speed));
    }
    else if (speed < 0) // back
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m1));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m1, -speed));
    }
    else // stop
    {
        if (stop_brake) ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
        else ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
    }
}

// 控制电机2速度，speed的输入范围：±PWM_MOTOR_MAX_VALUE
// Control motor 2 speed, speed input range: ±PWM_MOTOR_MAX_VALUE
static void PwmMotor_Set_Speed_M2(int speed)
{
    speed = PwmMotor_Ignore_Dead_Zone(speed);
    speed = PwmMotor_Limit_Speed(speed);

    if (speed > 0) // forward
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m2));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m2, speed));
    }
    else if (speed < 0) // back
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m2));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m2, -speed));
    }
    else // stop
    {
        if (stop_brake) ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
        else ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
    }
}

// 控制电机3速度，speed的输入范围：±PWM_MOTOR_MAX_VALUE
// Control motor 3 speed, speed input range: ±PWM_MOTOR_MAX_VALUE
static void PwmMotor_Set_Speed_M3(int speed)
{
    speed = PwmMotor_Ignore_Dead_Zone(speed);
    speed = PwmMotor_Limit_Speed(speed);

    if (speed > 0) // forward
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m3));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m3, speed));
    }
    else if (speed < 0) // back
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m3));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m3, -speed));
    }
    else // stop
    {
        if (stop_brake) ESP_ERROR_CHECK(bdc_motor_brake(motor_m3));
        else ESP_ERROR_CHECK(bdc_motor_coast(motor_m3));
    }
}

// 控制电机4速度，speed的输入范围：±PWM_MOTOR_MAX_VALUE
// Control motor 4 speed, speed input range: ±PWM_MOTOR_MAX_VALUE
static void PwmMotor_Set_Speed_M4(int speed)
{
    speed = PwmMotor_Ignore_Dead_Zone(speed);
    speed = PwmMotor_Limit_Speed(speed);

    if (speed > 0) // forward
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m4));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m4, speed));
    }
    else if (speed < 0) // back
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m4));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m4, -speed));
    }
    else // stop
    {
        if (stop_brake) ESP_ERROR_CHECK(bdc_motor_brake(motor_m4));
        else ESP_ERROR_CHECK(bdc_motor_coast(motor_m4));
    }
}



// 控制电机转动。speed输入范围：±PWM_MOTOR_MAX_VALUE
// Control motor rotation. speed input range: ±PWM_MOTOR_MAX_VALUE
void PwmMotor_Set_Speed_All(int speed_1, int speed_2, int speed_3, int speed_4)
{
    PwmMotor_Set_Speed_M1(speed_1);
    PwmMotor_Set_Speed_M2(speed_2);
    PwmMotor_Set_Speed_M3(speed_3);
    PwmMotor_Set_Speed_M4(speed_4);
}

// 通过电机ID号控制电机转动。speed输入范围：±PWM_MOTOR_MAX_VALUE
// Motor rotation is controlled by motor ID number. speed input range: ±PWM_MOTOR_MAX_VALUE
void PwmMotor_Set_Speed(motor_id_t motor_id, int speed)
{
    if (motor_id == MOTOR_ID_M1) PwmMotor_Set_Speed_M1(speed);
    else if (motor_id == MOTOR_ID_M2) PwmMotor_Set_Speed_M2(speed);
    else if (motor_id == MOTOR_ID_M3) PwmMotor_Set_Speed_M3(speed);
    else if (motor_id == MOTOR_ID_M4) PwmMotor_Set_Speed_M4(speed);
    else if (motor_id == MOTOR_ID_ALL)
    {
        PwmMotor_Set_Speed_M1(speed);
        PwmMotor_Set_Speed_M2(speed);
        PwmMotor_Set_Speed_M3(speed);
        PwmMotor_Set_Speed_M4(speed);
    }
}

// 停止电机
// Stop motor
void PwmMotor_Stop(motor_id_t motor_id, bool brake)
{
    if (brake)
    {
        if (motor_id == MOTOR_ID_M1) ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
        else if (motor_id == MOTOR_ID_M2) ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
        else if (motor_id == MOTOR_ID_M3) ESP_ERROR_CHECK(bdc_motor_brake(motor_m3));
        else if (motor_id == MOTOR_ID_M4) ESP_ERROR_CHECK(bdc_motor_brake(motor_m4));
        else if (motor_id == MOTOR_ID_ALL)
        {
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m3));
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m4));
        }
        stop_brake = true;
    }
    else
    {
        if (motor_id == MOTOR_ID_M1) ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
        else if (motor_id == MOTOR_ID_M2) ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
        else if (motor_id == MOTOR_ID_M3) ESP_ERROR_CHECK(bdc_motor_coast(motor_m3));
        else if (motor_id == MOTOR_ID_M4) ESP_ERROR_CHECK(bdc_motor_coast(motor_m4));
        else if (motor_id == MOTOR_ID_ALL)
        {
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m3));
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m4));
        }
        stop_brake = false;
    }
}

// 初始化电机
// Initial motor
void PwmMotor_Init(void)
{
    ESP_LOGI(TAG, "Init PwmMotor Device");

    PwmMotor_Init_M1();
    PwmMotor_Init_M2();
    PwmMotor_Init_M3();
    PwmMotor_Init_M4();
}

