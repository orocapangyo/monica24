#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flash.h"
#include "key.h"

static const char *TAG = "FLASH_MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    vTaskDelay(pdMS_TO_TICKS(100));

    Key_Init();
    Flash_Init();

    uint16_t value_u16 = 0;
    int32_t value_i32 = 0;
    uint16_t blob_len = 20;
    char* test_blob = malloc(blob_len);

    // 将数据从flash中读取出来，并打印输出。
    // Read the data from flash and print it out.
    Flash_Read_Data_UInt16(FKEY_TEST_UINT16, &value_u16);
    ESP_LOGI(TAG, "read u16 value: %d", value_u16);

    Flash_Read_Data_Int32(FKEY_TEST_INT32, &value_i32);
    ESP_LOGI(TAG, "read i32 value: %ld", value_i32);

    Flash_Read_Blob(FKEY_TEST_BLOB, (char*)test_blob, &blob_len);
    ESP_LOGI(TAG, "read blob: %s", test_blob);
 
    while (1)
    {
        if (Key0_Read_State() == KEY_STATE_PRESS)
        {
            ESP_LOGI(TAG, "KEY 0 PRESS");
            value_u16++;
            Flash_Write_Data_UInt16(FKEY_TEST_UINT16, value_u16);
            ESP_LOGI(TAG, "save u16 value: %d", value_u16);
        }
        if (Key1_Read_State() == KEY_STATE_PRESS)
        {
            ESP_LOGI(TAG, "KEY 1 PRESS");
            value_i32++;
            sprintf(test_blob, "blob:%ld", value_i32);
            Flash_Write_Data_Int32(FKEY_TEST_INT32, value_i32);
            Flash_Write_Blob(FKEY_TEST_BLOB, test_blob, blob_len);
            ESP_LOGI(TAG, "save flash blob: %s", test_blob);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}
