#include "lidar_ms200.h"

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"


#include "uart1.h"
#include "ms200.h"



static const char *TAG = "LIDAR_MS200";


ms200_data_t lidar_data = {0};


// 激光雷达解析数据任务
// Lidar data parsing mission
static void Lidar_Ms200_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Lidar_Ms200_Task with core:%d", xPortGetCoreID());
    uint16_t rx_count = 0;
    
    while (1)
    {
        rx_count = Uart1_Available();
        if (rx_count)
        {
            // Uart1_Clean_Buffer();
            for (int i = 0; i < rx_count; i++)
            {
                Ms200_Data_Receive(Uart1_Read());
            }
        }
        if (Ms200_New_Package())
        {
            Ms200_Clear_New_Package_State();
            Ms200_Get_Data(&lidar_data);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    vTaskDelete(NULL);
}

// 读取激光雷达某个点检测的距离
// Read the distance detected by the lidar at a point
uint16_t Lidar_Ms200_Get_Distance(uint16_t point)
{
    if (point < MS200_POINT_MAX)
    {
        return lidar_data.points[point].distance;
    }
    return 0;
}

// 读取激光雷达某个点检测的强度
// Read the intensity detected by the lidar at a point
uint16_t Lidar_Ms200_Get_Intensity(uint16_t point)
{
    if (point < MS200_POINT_MAX)
    {
        return lidar_data.points[point].intensity;
    }
    return 0;
}



// 初始化MS200激光雷达
// Initialize the MS200 Lidar
void Lidar_Ms200_Init(void)
{

    xTaskCreatePinnedToCore(Lidar_Ms200_Task, "Lidar_Ms200_Task", 10*1024, NULL, 10, NULL, 1);
}



