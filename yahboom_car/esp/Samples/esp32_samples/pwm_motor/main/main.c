#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pwm_motor.h"


static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    vTaskDelay(pdMS_TO_TICKS(1000));
    int speed = PWM_MOTOR_MAX_VALUE;
    PwmMotor_Init();

    while (1)
    {
        ESP_LOGI(TAG, "Motor Forward");
        PwmMotor_Set_Speed_All(speed, speed, speed, speed);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor Stop Brake");
        PwmMotor_Stop(MOTOR_ID_ALL, STOP_BRAKE);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor backward");
        PwmMotor_Set_Speed_All(-speed, -speed, -speed, -speed);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor Stop Coast");
        PwmMotor_Stop(MOTOR_ID_ALL, STOP_COAST);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
