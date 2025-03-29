#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"


static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    float speed_m1, speed_m2, speed_m3, speed_m4;
    int count = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));

    Motor_Init();
    Motor_Set_Speed(0.5, 0.5, -0.30, -0.30);

    while (1)
    {
        Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);
        ESP_LOGI(TAG, "Read_Speed:%.2f, %.2f, %.2f, %.2f", speed_m1, speed_m2, speed_m3, speed_m4);
        vTaskDelay(pdMS_TO_TICKS(100));
        count++;
        if(count >= 50)
        {
            Motor_Stop(STOP_COAST);
        }
    }
}
