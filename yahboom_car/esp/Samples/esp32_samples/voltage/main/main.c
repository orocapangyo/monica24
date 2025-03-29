#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "battery.h"

static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Battery_Init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        float voltage = Battery_Get_Voltage();
        ESP_LOGI(TAG, "Voltage:%.2fV", voltage);
    }
    
}
