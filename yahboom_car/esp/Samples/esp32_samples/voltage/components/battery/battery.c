#include "battery.h"

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "sdkconfig.h"

#define ADC_ATTEN_BATTERY      ADC_ATTEN_DB_11
#define ADC_CHANNEL_BATTERY    ADC_CHANNEL_2

const static char *TAG = "BATTERY";

static int adc_raw = 0;
static int cali_voltage = 0;
static float battery_voltage = 0.0;

adc_oneshot_unit_handle_t battery_handle = NULL;
adc_cali_handle_t battery_cali_handle = NULL;

// ADC校准初始化 ADC calibration initialization
static bool Battery_Adc_Calibration_Init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
    return calibrated;
}

// 注销ADC校准 Deregister ADC calibration
static void Battery_Adc_Calibration_Init_Deinit(adc_cali_handle_t handle)
{
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}

// 根据GPIO电压计算电池端电压 Calculate the battery voltage based on the GPIO voltage
static void Battery_Update_Voltage(int gpio_voltage_mV)
{
    // battery_voltage = gpio_voltage*(10+3.3)/3.3;
    battery_voltage = gpio_voltage_mV / 1000.0 * 4.03;
}

// 电池ADC初始化 Battery ADC initialization
static void Battery_Adc_Init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &battery_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_BATTERY,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(battery_handle, ADC_CHANNEL_BATTERY, &config));

    Battery_Adc_Calibration_Init(ADC_UNIT_1, ADC_CHANNEL_BATTERY, ADC_ATTEN_BATTERY, &battery_cali_handle);
}

// 电池电压读取任务 Battery voltage reading task
static void Battery_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Battery_Task with core:%d", xPortGetCoreID());

    while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(battery_handle, ADC_CHANNEL_BATTERY, &adc_raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(battery_cali_handle, adc_raw, &cali_voltage));
        Battery_Update_Voltage(cali_voltage);
        // ESP_LOGI(TAG, "Raw Data: %d, Calibration voltage: %d", adc_raw, cali_voltage);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 结束任务前注销ADC Deregister ADC before ending the task
    ESP_ERROR_CHECK(adc_oneshot_del_unit(battery_handle));
    Battery_Adc_Calibration_Init_Deinit(battery_cali_handle);
    vTaskDelete(NULL);
}

// 初始化电池 Initial Battery
void Battery_Init(void)
{
    Battery_Adc_Init();

    // 开启电池电压读取任务 Enable the battery voltage reading task
    xTaskCreatePinnedToCore(Battery_Task, "Battery_Task", 3 * 1024, NULL, 2, NULL, 1);
}

// 读取电池电压, 单位:V Read the battery voltage, unit :V
float Battery_Get_Voltage(void)
{
    return battery_voltage;
}
