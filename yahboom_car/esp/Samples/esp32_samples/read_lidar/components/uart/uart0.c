#include "uart0.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "ring_buffer.h"


static const char *TAG = "UART0";

Ring_Buffer_t uart0_ringbuf;


// 串口0接收和发送任务 Serial port 0 Receives and sends tasks
static void Uart0_Rx_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Uart0_Rx_Task with core:%d", xPortGetCoreID());
    uint16_t temp_len = 255;
    uint8_t* temp_data = (uint8_t*) malloc(temp_len);

    while (1)
    {
        // 从串口0读取数据，并将读取的数据通过串口0发送出去。
        // The data is read from serial port 0 and sent out through serial port 0.
        const int rxBytes = uart_read_bytes(UART_NUM_0, temp_data, temp_len, 1 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            for (int i = 0; i < rxBytes; i++)
            {
                RingBuffer_Push(&uart0_ringbuf, temp_data[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(temp_data);
    vTaskDelete(NULL);
}


// 初始化串口0, 波特率为115200 
// Initialize serial port 0, the baud rate is 115200.
void Uart0_Init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, RX0_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART0_GPIO_TXD, UART0_GPIO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    RingBuffer_Init(&uart0_ringbuf, RX0_BUF_SIZE);

    xTaskCreate(Uart0_Rx_Task, "Uart0_Rx_Task", 5*1024, NULL, configMAX_PRIORITIES, NULL);
}

// 通过串口0发送一串数据 Send a string of data through serial port 0
int Uart0_Send_Data(uint8_t* data, uint16_t len)
{
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    return txBytes;
}

// 通过串口0发送一个字节 Send a byte through serial port 0
int Uart0_Send_Byte(uint8_t data)
{
    uint8_t data1 = data;
    const int txBytes = uart_write_bytes(UART_NUM_0, &data1, 1);
    return txBytes;
}

// 返回串口0缓存数据的数量
// Return the amount of cached data in serial port 0
uint16_t Uart0_Available(void)
{
    return RingBuffer_Get_Used_Count(&uart0_ringbuf);
}

// 从串口0缓存数据中读取一个字节
// Reads a byte from serial port 0 cache data
uint8_t Uart0_Read(void)
{
    return RingBuffer_Pop(&uart0_ringbuf);
}

// 清除串口0的缓存数据
// Clear cache data from serial port 0
void Uart0_Clean_Buffer(void)
{
    RingBuffer_Clean_Queue(&uart0_ringbuf);
}

