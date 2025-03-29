#include <stdio.h>
#include "esp_log.h"
#include "key.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"


static const char *TAG = "KEY_MAIN";

void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Key_Init();

    while (1)
    {
        if (Key0_Read_State() == KEY_STATE_PRESS)
        {
            ESP_LOGI(TAG, "KEY 0 PRESS");
            ESP_LOGI(TAG, "free internal heap size = %ld", esp_get_free_internal_heap_size());
        }
        if (Key1_Read_State() == KEY_STATE_PRESS)
        {
            ESP_LOGI(TAG, "KEY 1 PRESS");
            printf("free heap size = %ld\r\n", esp_get_free_heap_size());
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
