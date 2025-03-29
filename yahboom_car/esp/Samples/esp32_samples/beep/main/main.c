#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "beep.h"

static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    Beep_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 每1秒响一次蜂鸣器 The buzzer sounds every 1 second
    while(1)
    {
        Beep_On_Time(500);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
