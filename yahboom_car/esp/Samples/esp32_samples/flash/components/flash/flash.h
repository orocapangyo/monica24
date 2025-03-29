#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"


#define STORAGE_NAMESPACE         "YAHBOOM"

#define FKEY_TEST_INT32           "TEST_INT32"
#define FKEY_TEST_UINT16          "TEST_UINT16"
#define FKEY_TEST_BLOB            "TEST_BLOB"


void Flash_Init(void);

esp_err_t Flash_Read_Data_Int8(const char* key, int8_t* out_value);
esp_err_t Flash_Write_Data_Int8(const char* key, int8_t value);

esp_err_t Flash_Read_Data_UInt16(const char* key, uint16_t* out_value);
esp_err_t Flash_Write_Data_UInt16(const char* key, uint16_t value);


esp_err_t Flash_Read_Data_Int32(const char* key, int32_t* out_value);
esp_err_t Flash_Write_Data_Int32(const char* key, int32_t value);


esp_err_t Flash_Read_Blob(const char* key, char* out_data, uint16_t* length);
esp_err_t Flash_Write_Blob(const char* key, char* in_data, uint16_t length);


#ifdef __cplusplus
}
#endif
