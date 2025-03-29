#include "beep.h"


#include "stdio.h"
#include "stdint.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"



const static char *TAG = "BEEP";



static uint8_t beep_state = BEEP_STATE_OFF;
static uint16_t beep_on_time = 0;


// 配置蜂鸣器GPIO口  configure GPIO with the buzzer
static void Beep_GPIO_Init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt 禁用中断
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode 设置为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set 引脚编号设置
    io_conf.pin_bit_mask = (1ULL<<BEEP_GPIO);
    //disable pull-down mode 禁用下拉
    io_conf.pull_down_en = 0;
    //disable pull-up mode 禁用上拉
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings 配置GPIO口
    gpio_config(&io_conf);
    // 关闭蜂鸣器
    Beep_Off();
}

// 蜂鸣器后台任务 Buzzer background task
static void Beep_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Beep_Task with core:%d", xPortGetCoreID());
    while (1)
    {
        Beep_Handle();

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}


// 初始化蜂鸣器
void Beep_Init(void)
{
    Beep_GPIO_Init();

    // 开启蜂鸣器任务 start buzzer task
    xTaskCreatePinnedToCore(Beep_Task, "Beep_Task", 3*1024, NULL, 5, NULL, 1);
}

// 打开蜂鸣器 turn on buzzer
void Beep_On(void)
{
    beep_state = BEEP_STATE_ON_ALWAYS;
    beep_on_time = 0;
    gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
}

// 关闭蜂鸣器 turn off buzzer
void Beep_Off(void)
{
    beep_state = BEEP_STATE_OFF;
    beep_on_time = 0;
    gpio_set_level(BEEP_GPIO, !BEEP_ACTIVE_LEVEL);
}

// 设置蜂鸣器开启时间，time=0时关闭，time=1时一直响，time>=10，延迟xx毫秒后自动关闭
// Set the buzzer start time. When time=0, the buzzer is turned off. 
// When time=1, the buzzer keeps ringing. 
// When time>=10, the buzzer is turned off automatically after xx milliseconds
void Beep_On_Time(uint16_t time)
{
	if (time == 1)
	{
		beep_state = BEEP_STATE_ON_ALWAYS;
		beep_on_time = 0;
		Beep_On();
	}
	else if (time == 0)
	{
		beep_state = BEEP_STATE_OFF;
		beep_on_time = 0;
		Beep_Off();
	}
	else
	{
		if (time < 10) time = 10;
        if (time > 10000) time = 10000;
        beep_state = BEEP_STATE_ON_DELAY;
        beep_on_time = (time / 10);
        gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
	}
}

// 蜂鸣器自动停止控制器，每10毫秒调用一次。
// The buzzer automatically stops the controller and is called every 10 milliseconds.
void Beep_Handle(void)
{
    if (beep_state == BEEP_STATE_ON_DELAY)
    {
        if (beep_on_time > 0)
        {
            beep_on_time--;
        }
        else
        {
            Beep_Off();
            beep_state = BEEP_STATE_OFF;
        }
    }
}
