#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart1.h"
#include "lidar_ms200.h"


static const char *TAG = "LIDAR_MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Uart1_Init();
    Lidar_Ms200_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint16_t points[4] = {0, 90, 180, 270};
    uint16_t distances[4] = {0};
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            distances[i] = Lidar_Ms200_Get_Distance(points[i]);
        }
        ESP_LOGI(TAG, "distance:%d, %d, %d, %d", distances[0], distances[1], distances[2], distances[3]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
