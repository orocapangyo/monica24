#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"

// I2C主机相关参数配置 I2C master parameters
#define I2C_MASTER_GPIO_SCL         39
#define I2C_MASTER_GPIO_SDA         40
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000


void I2C_Master_Init(void);
void I2C_Master_Delete(void);

esp_err_t I2C_Master_Read(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* data);
uint8_t I2C_Master_Read_Byte(uint8_t addr, uint8_t reg);

esp_err_t I2C_Master_Write(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* data);
esp_err_t I2C_Master_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);


#ifdef __cplusplus
}
#endif
