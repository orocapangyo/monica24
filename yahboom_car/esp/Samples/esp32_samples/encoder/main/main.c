#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder.h"


static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Encoder_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        ESP_LOGI(TAG, "Encoder:%d, %d, %d, %d", 
            Encoder_Get_Count_M1(), Encoder_Get_Count_M2(), 
            Encoder_Get_Count_M3(), Encoder_Get_Count_M4());

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
