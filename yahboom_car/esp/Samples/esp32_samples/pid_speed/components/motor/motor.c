#include "motor.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"


#include "pwm_motor.h"
#include "encoder.h"
#include "pid_ctrl.h"



static const char *TAG = "MOTOR";


// 10毫秒目标脉冲数
// 10 ms target pulse number
static float speed_count[MOTOR_MAX_NUM] = {0};

// 通过编码器计算得到电机速度，单位:m/s
// The motor speed is calculated by the encoder, unit :m/s
static float read_speed[MOTOR_MAX_NUM] = {0};

// PID参数结构体
// PID parameter structure
pid_ctrl_parameter_t pid_runtime_param = {0};

// PID电机控制器
// PID motor controller
pid_ctrl_block_handle_t pid_motor[MOTOR_MAX_NUM];


// PID计算后输出的速度值
// PID Output speed value after calculation
static float new_pid_output[MOTOR_MAX_NUM] = {0};
static float pid_target[MOTOR_MAX_NUM] = {0};
static float pid_enable = 0;

static float Motor_Limit_Speed(float speed)
{
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

// PID算法控制电机速度
// PID algorithm controls motor speed
static void Motor_PID_Ctrl(void)
{
    static int last_count[MOTOR_MAX_NUM] = {0};
    static int cur_count[MOTOR_MAX_NUM] = {0};
    static float real_pulse[MOTOR_MAX_NUM] = {0};
    static float new_speed[MOTOR_MAX_NUM] = {0};

    for (int i = 0; i < MOTOR_MAX_NUM; i++)
    {
        cur_count[i] = Encoder_Get_Count(ENCODER_ID_M1 + i);
        real_pulse[i] = cur_count[i] - last_count[i];
        last_count[i] = cur_count[i];
        read_speed[i] = real_pulse[i] * (MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD);
        if (pid_enable)
        {
            pid_compute(pid_motor[i], pid_target[i] - real_pulse[i], &new_speed[i]);
            PwmMotor_Set_Speed(MOTOR_ID_M1 + i, (int)new_speed[i]);
            new_pid_output[i] = new_speed[i];
        }
    }
}


static void Motor_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Motor_Task with core:%d", xPortGetCoreID());
    pid_runtime_param.kp = 1.0;
    pid_runtime_param.ki = 0.2;
    pid_runtime_param.kd = 0.2;
    pid_runtime_param.cal_type = PID_CAL_TYPE_INCREMENTAL;
    pid_runtime_param.max_output   = PWM_MOTOR_MAX_VALUE;
    pid_runtime_param.min_output   = -PWM_MOTOR_MAX_VALUE;
    pid_runtime_param.max_integral = 1000;
    pid_runtime_param.min_integral = -1000;
    
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    for (int i = 0; i < MOTOR_MAX_NUM; i++)
    {
        ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_motor[i]));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        Motor_PID_Ctrl();
        vTaskDelayUntil(&lastWakeTime, MOTOR_PID_PERIOD);
    }

    Motor_Stop(STOP_BRAKE);
    vTaskDelete(NULL);
}

// 设置电机速度，单位：m/s
// Set motor speed, unit: m/s
void Motor_Set_Speed(float speed_m1, float speed_m2, float speed_m3, float speed_m4)
{
    static float speed_m[MOTOR_MAX_NUM] = {0};
    speed_m[0] = Motor_Limit_Speed(speed_m1);
    speed_m[1] = Motor_Limit_Speed(speed_m2);
    speed_m[2] = Motor_Limit_Speed(speed_m3);
    speed_m[3] = Motor_Limit_Speed(speed_m4);

    for (int i = 0; i < MOTOR_MAX_NUM; i++)
    {
        // 速度转化成10毫秒编码器目标数量
        // The speed is converted to the number of encoder targets in 10 milliseconds
        speed_count[i] = speed_m[i] / (MOTOR_WHEEL_CIRCLE/MOTOR_ENCODER_CIRCLE/MOTOR_PID_PERIOD);
        pid_target[i] = (float)speed_count[i];
    }
    pid_enable = 1;
}

// 读取当前电机速度值
// Read the current motor speed value
void Motor_Get_Speed(float* speed_m1, float* speed_m2, float* speed_m3, float* speed_m4)
{
    *speed_m1 = read_speed[0];
    *speed_m2 = read_speed[1];
    *speed_m3 = read_speed[2];
    *speed_m4 = read_speed[3];
}


// 电机停止，brake=true表示刹车停止，brake=false表示滑行停止。
// The motor stops. brake=true indicates that the brake stops, and brake=false indicates that the coasting stops.
void Motor_Stop(bool brake)
{
    Motor_Set_Speed(0, 0, 0, 0);
    pid_enable = 0;
    PwmMotor_Stop(MOTOR_ID_ALL, brake);
}

// 更新电机PID参数
// Update motor PID parameters
void Motor_Update_PID_Parm(float pid_p, float pid_i, float pid_d)
{
    pid_runtime_param.kp = pid_p;
    pid_runtime_param.ki = pid_i;
    pid_runtime_param.kd = pid_d;
    for (int i = 0; i < MOTOR_MAX_NUM; i++)
    {
        pid_update_parameters(pid_motor[i], &pid_runtime_param);
    }
}

// 读取电机PID参数
// Read motor PID parameters
void Motor_Read_PID_Parm(float* out_p, float* out_i, float* out_d)
{
    *out_p = pid_runtime_param.kp;
    *out_i = pid_runtime_param.ki;
    *out_d = pid_runtime_param.kd;
}

// 初始化编码器电机
// Initialize the encoder motor
void Motor_Init(void)
{
    Encoder_Init();
    PwmMotor_Init();

    xTaskCreatePinnedToCore(Motor_Task, "Motor_Task", 10*1024, NULL, 10, NULL, 1);
}

