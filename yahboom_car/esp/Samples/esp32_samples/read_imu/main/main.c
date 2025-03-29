#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icm42670p.h"

// 控制打印原始16进制数据
// Control to print raw hexadecimal data
#define ENABLE_DEBUG_IMU_RAW       0


static const char *TAG = "IMU_MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");

    Icm42670p_Init();
    #if ENABLE_DEBUG_IMU_RAW
    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};
    #else
    float accel_g[3] = {0};
    float gyro_dps[3] = {0};
    #endif

    while (1)
    {
        if (Icm42670p_Start_OK() > 0)
        {
            #if ENABLE_DEBUG_IMU_RAW
            Icm42670p_Get_Accel_RawData(accel_raw);
            Icm42670p_Get_Gyro_RawData(gyro_raw);
            ESP_LOGI(TAG, "Raw Accel:%d, %d, %d, Gyro:%d, %d, %d", 
                accel_raw[0], accel_raw[1], accel_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]);
            #else
            Icm42670p_Get_Accel_g(accel_g);
            Icm42670p_Get_Gyro_dps(gyro_dps);
            ESP_LOGI(TAG, "Accel_g:%f, %f, %f, Gyro_dps:%f, %f, %f", 
                accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
}
