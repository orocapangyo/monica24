#include "key.h"

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"



// 配置按键GPIO口  configure GPIO with the key
static void Key_GPIO_Init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt 禁用中断
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode 设置为输入模式
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set 引脚编号设置
    io_conf.pin_bit_mask = ((1ULL<<KEY_GPIO_BOOT0)|(1ULL<<KEY_GPIO_USER1));
    //disable pull-down mode 禁用下拉
    io_conf.pull_down_en = 0;
    //enable pull-up mode 使能上拉
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings 配置GPIO口
    gpio_config(&io_conf);
}

// 判断按键key0是否被按下，如果被按下返回KEY_STATE_PRESS，松开状态返回KEY_STATE_RELEASE
// Check whether the key0 is pressed. If pressed, KEY_STATE_PRESS is returned. If released, KEY_STATE_RELEASE is returned
static uint8_t Key0_is_Pressed(void)
{
    uint8_t key_state = KEY_STATE_RELEASE;
    if (!gpio_get_level(KEY_GPIO_BOOT0))
    {
        key_state = KEY_STATE_PRESS;
    }
    return key_state;
}

// 判断按键key1是否被按下，如果被按下返回KEY_STATE_PRESS，松开状态返回KEY_STATE_RELEASE
// Check whether the key1 is pressed. If pressed, KEY_STATE_PRESS is returned. If released, KEY_STATE_RELEASE is returned
static uint8_t Key1_is_Pressed(void)
{
    uint8_t key_state = KEY_STATE_RELEASE;
    if (!gpio_get_level(KEY_GPIO_USER1))
    {
        key_state = KEY_STATE_PRESS;
    }
    return key_state;
}



// 非阻塞式读取按键key0的状态（软件防抖）, 每次按键按下返回一次KEY_STATE_PRESS
// Non-blocking read key0 status (software anti-shake), Returns the KEY_STATE_PRESS once per key press
uint8_t Key0_Read_State(void)
{
	static uint16_t key_state = 0;

	if (Key0_is_Pressed() == KEY_STATE_PRESS)
	{
		if (key_state < 3)
		{
			key_state++;
		}
	}
	else
	{
		key_state = 0;
	}
	if (key_state == 2)
	{
		return KEY_STATE_PRESS;
	}
	return KEY_STATE_RELEASE;
}

// 非阻塞式读取按键key1的状态（软件防抖）, 每次按键按下返回一次KEY_STATE_PRESS
// Non-blocking read key1 status (software anti-shake), Returns the KEY_STATE_PRESS once per key press
uint8_t Key1_Read_State(void)
{
	static uint16_t key_state = 0;

	if (Key1_is_Pressed() == KEY_STATE_PRESS)
	{
		if (key_state < 3)
		{
			key_state++;
		}
	}
	else
	{
		key_state = 0;
	}
    if (key_state == 2)
	{
		return KEY_STATE_PRESS;
	}
	return KEY_STATE_RELEASE;
}


// 初始化按键 Initialize key
void Key_Init(void)
{
    Key_GPIO_Init();
}
