#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo.h"

static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Servo_Init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1) 
    {
        ESP_LOGI(TAG, "Servo_Set_Angle:-90");
        Servo_Set_Angle(SERVO_ID_S1, -90);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Servo_Set_Angle:90");
        Servo_Set_Angle(SERVO_ID_S1, 90);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
