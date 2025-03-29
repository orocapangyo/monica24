#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"

static const char *TAG = "LED_MAIN";

void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    Led_Init();

    while (1)
    {
        Led_Flash(50);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
