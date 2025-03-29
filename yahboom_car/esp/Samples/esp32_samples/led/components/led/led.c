#include "led.h"

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


// LED闪烁，需每10毫秒调用一次。interval表示间隔的时间，单位为10ms。
// The LED blinks and needs to be called every 10 milliseconds. interval Indicates the interval (unit: 10ms).
void Led_Flash(uint16_t interval)
{
    static uint16_t state = 0;
    static uint16_t count = 0;
    count++;
    if (count >= interval)
    {
        count = 0;
        state = (state + 1) % 2;
        Led_State(state);
    }
}


// 初始化LED
void Led_Init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt 禁用中断
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode 设置为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set 引脚编号设置
    io_conf.pin_bit_mask = (1ULL<<LED_GPIO);
    //disable pull-down mode 禁用下拉
    io_conf.pull_down_en = 0;
    //disable pull-up mode 禁用上拉
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings 配置GPIO口
    gpio_config(&io_conf);
    // 关闭LED灯
    Led_Off();
}

// 打开LED灯 turn on led
void Led_On(void)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
}

// 关闭LED灯 turn off led
void Led_Off(void)
{
    gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
}

// 修改LED灯状态，state=1表示点亮，state=0表示熄灭。
// Change the LED status. state=1 indicates on, and state=0 indicates off.
void Led_State(uint8_t state)
{
    if (state == 0)
    {
        gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
    }
    else
    {
        gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
    }
}
