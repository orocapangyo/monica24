#include "servo.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_log.h"
#include "driver/mcpwm_prelude.h"



mcpwm_cmpr_handle_t comparator_S1 = NULL;
mcpwm_cmpr_handle_t comparator_S2 = NULL;


// 舵机S1角度转成脉冲信号, 控制舵机角度，限制并矫正角度输入最大值与最小值。
// The servo S1 angle turns into a pulse signal, Control the servo Angle, limit and correct the Angle input maximum and minimum values.
static uint32_t Servo_S1_Angle_To_Compare(int8_t angle)
{
    if (angle > SERVO_S1_MAX_ANGLE) angle = SERVO_S1_MAX_ANGLE;
    if (angle < SERVO_S1_MIN_ANGLE) angle = SERVO_S1_MIN_ANGLE;
    int cmp = ((int)angle - SERVO_MIN_HD_ANGLE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_HD_ANGLE - SERVO_MIN_HD_ANGLE) + SERVO_MIN_PULSEWIDTH_US;
    return (uint32_t)cmp;
}

// 舵机S2角度转成脉冲信号, 控制舵机角度，限制并矫正角度输入最大值与最小值。
// The servo S2 angle turns into a pulse signal, Control the servo Angle, limit and correct the Angle input maximum and minimum values.
static uint32_t Servo_S2_Angle_To_Compare(int8_t angle)
{
    if (angle > SERVO_S2_MAX_ANGLE) angle = SERVO_S2_MAX_ANGLE;
    if (angle < SERVO_S2_MIN_ANGLE) angle = SERVO_S2_MIN_ANGLE;
    int cmp = ((int)angle - SERVO_MIN_HD_ANGLE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_HD_ANGLE - SERVO_MIN_HD_ANGLE) + SERVO_MIN_PULSEWIDTH_US;
    return (uint32_t)cmp;
}


// 配置舵机S1的PWM输出信息
static void Servo_S1_Init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = SERVO_TIMER_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = SERVO_TIMER_GROUP_ID, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_S1));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO_S1,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    uint32_t cmp1 = Servo_S1_Angle_To_Compare(SERVO_S1_DEF_ANGLE);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_S1, cmp1));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_S1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void Servo_S2_Init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = SERVO_TIMER_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = SERVO_TIMER_GROUP_ID, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_S2));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_GPIO_S2,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    uint32_t cmp2 = Servo_S2_Angle_To_Compare(SERVO_S2_DEF_ANGLE);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_S2, cmp2));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_S2, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}



// 控制舵机转动的角度, 根据设置的最大值和最小值范围控制angle的值。
// Control the rotation angle of the servo. The value of angle is controlled according to the range of maximum and minimum values set.
void Servo_Set_Angle(servo_id_t servo_id, int8_t angle)
{
    int8_t angle_servo = 0;
    if (servo_id == SERVO_ID_S1)
    {
        angle_servo = -angle;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_S1, Servo_S1_Angle_To_Compare(angle_servo)));
        return;
    }
    if (servo_id == SERVO_ID_S2)
    {
        angle_servo = angle;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_S2, Servo_S2_Angle_To_Compare(angle_servo)));
        return;
    }
}


// 初始化PWM舵机 Initializing pwm servo 
void Servo_Init(void)
{
    Servo_S1_Init();
    Servo_S2_Init();
}

