#include "serial_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"


#include <rmw_microros/rmw_microros.h>
#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>


#define ROS_UART_NUM       CONFIG_MICRO_ROS_UART_NUM
#define ROS_UART_BAUD      CONFIG_MICRO_ROS_UART_BAUD
#define UART_TXD           CONFIG_MICRO_ROS_UART_TXD
#define UART_RXD           CONFIG_MICRO_ROS_UART_RXD
#define UART_RTS           CONFIG_MICRO_ROS_UART_RTS
#define UART_CTS           CONFIG_MICRO_ROS_UART_CTS


#define UART_BUFFER_SIZE (512)

static int32_t serial_baudrate = ROS_UART_BAUD;

// 打开串口
// Open the serial port
static bool transport_serial_open(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;

    uart_config_t uart_config = {
        .baud_rate = serial_baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL)
    {
        return false;
    }
    if (uart_set_pin(*uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) == ESP_FAIL)
    {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL)
    {
        return false;
    }
    return true;
}

// 关闭串口
// Close the serial port
static bool transport_serial_close(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_driver_delete(*uart_port) == ESP_OK;
}

// 向串口写入数据
// Writes data to the serial port
static size_t transport_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char *)buf, len);
    return txBytes;
}

// 从串口读取数据
// Read data from the serial port
static size_t transport_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rxBytes;
}


// 设置自定义传输方式为串口传输方式
// Set the custom transmission mode to serial port transmission
rmw_ret_t set_microros_serial_transports(void)
{
    rmw_ret_t ret = RMW_RET_OK;
    static size_t uart_port = ROS_UART_NUM;
    ret = rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        transport_serial_open,
        transport_serial_close,
        transport_serial_write,
        transport_serial_read
    );
    return ret;
}

// 设置自定义传输方式为串口传输方式，带RMW选项
// Set the custom transmission mode to serial port transmission with RMW option
rmw_ret_t set_microros_serial_transports_with_options(rmw_init_options_t * rmw_options)
{
    rmw_ret_t ret = RMW_RET_OK;
    static size_t uart_port = ROS_UART_NUM;
    uart_driver_delete(UART_NUM_0);
    ret = rmw_uros_options_set_custom_transport(
        true,
        (void *) &uart_port,
        transport_serial_open,
        transport_serial_close,
        transport_serial_write,
        transport_serial_read,
        rmw_options
    );
    return ret;
}

