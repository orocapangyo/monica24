#include "flash.h"

#include "stdio.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"


#define ENABLE_DEBUG_FLASH         0
#if ENABLE_DEBUG_FLASH
#define YB_DEBUG    ESP_LOGI
#else
#define YB_DEBUG    ESP_LOGV
#endif

static const char *TAG = "FLASH";


// 从flash芯片读取一个int8类型数据
// Read an int8 type data from the flash chip
esp_err_t Flash_Read_Data_Int8(const char* key, int8_t* out_value)
{
    nvs_handle_t my_handle;
    int8_t read_value = 0;
    esp_err_t err = ESP_OK;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_get_i8(my_handle, key, &read_value);
    if (err == ESP_OK)
    {
        *out_value = read_value;
    }
    else
    {
        *out_value = 0;
    }
    nvs_close(my_handle);
    return err;
}

// 写入一个uint8类型数据到flash芯片
// Write a uint8 type data to the flash chip
esp_err_t Flash_Write_Data_Int8(const char* key, int8_t value)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_FAIL;
    int8_t read_value = 0;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_set_i8(my_handle, key, value);
    // 写入数据后必须调用nvs_commit提交写入闪存。
    // After writing data, you must call nvs commit to write to flash.
    if (err == ESP_OK) err = nvs_commit(my_handle);
    if (err == ESP_OK) err = nvs_get_i8(my_handle, key, &read_value);
    if (err == ESP_OK && value != read_value) err = ESP_FAIL;
    YB_DEBUG(TAG, "%s %d, Write int8:%d, Read:%d", key, err, value, read_value);
    nvs_close(my_handle);
    return err;
}


// 从flash芯片读取一个uint16类型数据
// Read a uint16 type data from the flash chip
esp_err_t Flash_Read_Data_UInt16(const char* key, uint16_t* out_value)
{
    nvs_handle_t my_handle;
    uint16_t read_value = 0;
    esp_err_t err = ESP_OK;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_get_u16(my_handle, key, &read_value);
    if (err == ESP_OK)
    {
        *out_value = read_value;
    }
    else
    {
        *out_value = 0;
    }
    nvs_close(my_handle);
    return err;
}

// 写入一个uint16类型数据到flash芯片
// Write a uint16 type data to the flash chip
esp_err_t Flash_Write_Data_UInt16(const char* key, uint16_t value)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_FAIL;
    uint16_t read_value = 0;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(my_handle, key, value);
    // 写入数据后必须调用nvs_commit提交写入闪存。
    // After writing data, you must call nvs commit to write to flash.
    if (err == ESP_OK) err = nvs_commit(my_handle);
    if (err == ESP_OK) err = nvs_get_u16(my_handle, key, &read_value);
    if (err == ESP_OK && value != read_value) err = ESP_FAIL;
    YB_DEBUG(TAG, "%s %d, Write u16:%d, Read:%d", key, err, value, read_value);
    nvs_close(my_handle);
    return err;
}


// 从flash芯片读取一个int32类型数据
// Read an int32 type data from the flash chip
esp_err_t Flash_Read_Data_Int32(const char* key, int32_t* out_value)
{
    nvs_handle_t my_handle;
    int32_t read_value = 0;
    esp_err_t err = ESP_OK;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_get_i32(my_handle, key, &read_value);
    if (err == ESP_OK)
    {
        *out_value = read_value;
    }
    else
    {
        *out_value = 0;
    }
    nvs_close(my_handle);
    return err;
}

// 写入一个int32类型数据到flash芯片
// Write an int32 type data to the flash chip
esp_err_t Flash_Write_Data_Int32(const char* key, int32_t value)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_FAIL;
    int32_t read_value = 0;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_set_i32(my_handle, key, value);
    // 写入数据后必须调用nvs_commit提交写入闪存。
    // After writing data, you must call nvs commit to write to flash.
    if (err == ESP_OK) err = nvs_commit(my_handle);
    if (err == ESP_OK) err = nvs_get_i32(my_handle, key, &read_value);
    if (err == ESP_OK && value != read_value) err = ESP_FAIL;
    YB_DEBUG(TAG, "%s %d, Write int32:%ld, Read:%ld", key, err, value, read_value);
    nvs_close(my_handle);
    return err;
}


// 写入一块数据到flash芯片
// Write a piece of data to the flash chip
esp_err_t Flash_Read_Blob(const char* key, char* out_data, uint16_t* length)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;
    size_t read_length = *length;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_get_blob(my_handle, key, out_data, &read_length);
    *length = read_length;
    nvs_close(my_handle);
    YB_DEBUG(TAG, "%s %d, Read:%s, len:%d", key, err, out_data, read_length);
    return err;
}

// 从flash芯片读取一块数据
// Read a piece of data from the flash chip
esp_err_t Flash_Write_Blob(const char* key, char* in_data, uint16_t length)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;
    size_t write_length = length;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(my_handle, key, in_data, write_length);
    // 写入数据后必须调用nvs_commit提交写入闪存。
    // After writing data, you must call nvs commit to write to flash.
    if (err == ESP_OK) err = nvs_commit(my_handle);
    YB_DEBUG(TAG, "%s %d, Write:%s, len:%d", key, err, in_data, write_length);
    nvs_close(my_handle);
    return err;
}

void Flash_Init(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}
