#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "car_motion.h"


static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    int count = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));

    
    Motor_Init();
    Motion_Ctrl(0.5, 0, 0);
    car_motion_t micro_car;

    while (1)
    {
        Motion_Get_Speed(&micro_car);
        ESP_LOGI(TAG, "Read Motion:%.2f, %.2f, %.2f", micro_car.Vx, micro_car.Vy, micro_car.Wz);
        vTaskDelay(pdMS_TO_TICKS(100));
        count++;
        if(count >= 50)
        {
            Motion_Stop(STOP_COAST);
        }
    }
}
