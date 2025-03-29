#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart0.h"
#include "uart1.h"

static const char *TAG = "MAIN";


void app_main(void)
{
    printf("hello yahboom\n");
    ESP_LOGI(TAG, "Nice to meet you!");
    vTaskDelay(pdMS_TO_TICKS(100));

    Uart0_Init();
    Uart1_Init();

    uint16_t uart0_rx_len = 0;
    uint16_t uart1_rx_len = 0;
    int i = 0;

    while (1)
    {
        uart0_rx_len = Uart0_Available();
        if (uart0_rx_len)
        {
            for (i = 0; i < uart0_rx_len; i++)
            {
                Uart0_Send_Byte(Uart0_Read());
            }
        }

        uart1_rx_len = Uart1_Available();
        if (uart1_rx_len)
        {
            for (i = 0; i < uart1_rx_len; i++)
            {
                Uart1_Send_Byte(Uart1_Read());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
