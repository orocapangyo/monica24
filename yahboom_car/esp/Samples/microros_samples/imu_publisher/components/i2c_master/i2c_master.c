#include "i2c_master.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"


static const char *TAG = "I2C_MASTER";


// 初始化I2C硬件模块为主机模式，并配置GPIO口
// Initialize the I2C hardware module to master mode and configure the GPIO port
void I2C_Master_Init(void)
{
    ESP_LOGI(TAG, "Init I2C master:SCL->GPIO%d, SDA->GPIO%d", I2C_MASTER_GPIO_SCL, I2C_MASTER_GPIO_SDA);
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_GPIO_SDA,
        .scl_io_num = I2C_MASTER_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

// 注销I2C模块
// Deregister the I2C module
void I2C_Master_Delete(void)
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
}

// I2C读取一串数据
// I2C reads a string of data
esp_err_t I2C_Master_Read(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* data)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// I2C读取一个字节
// I2C reads a byte
uint8_t I2C_Master_Read_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t data = 0;
    i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return data;
}

// I2C写入一串数据
// I2C writes a string of data
esp_err_t I2C_Master_Write(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* data)
{
    int ret;
    uint8_t *buf = (uint8_t *)malloc(len+1);
    buf[0] = reg;
    for (int i = 0; i < len; i++)
    {
        buf[i+1] = data[i];
    }
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, len+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    free(buf);
    return ret;
}

// I2C写入一个字节
// I2C writes a byte
esp_err_t I2C_Master_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    int ret;
    uint8_t write_buf[] = {reg, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

