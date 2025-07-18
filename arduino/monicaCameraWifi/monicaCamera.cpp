#include <Arduino.h>
#include "monicaCamera.h"
#include "camera_pinmap.h"

MonicaCamera::MonicaCamera()
{

}

void MonicaCamera::initialize()
{
    config_.ledc_channel = LEDC_CHANNEL_0;
    config_.ledc_timer = LEDC_TIMER_0;
    config_.pin_d0 = Y2_GPIO_NUM;
    config_.pin_d1 = Y3_GPIO_NUM;
    config_.pin_d2 = Y4_GPIO_NUM;
    config_.pin_d3 = Y5_GPIO_NUM;
    config_.pin_d4 = Y6_GPIO_NUM;
    config_.pin_d5 = Y7_GPIO_NUM;
    config_.pin_d6 = Y8_GPIO_NUM;
    config_.pin_d7 = Y9_GPIO_NUM;
    config_.pin_xclk = XCLK_GPIO_NUM;
    config_.pin_pclk = PCLK_GPIO_NUM;
    config_.pin_vsync = VSYNC_GPIO_NUM;
    config_.pin_href = HREF_GPIO_NUM;
    config_.pin_sccb_sda = SIOD_GPIO_NUM;
    config_.pin_sccb_scl = SIOC_GPIO_NUM;
    config_.pin_pwdn = PWDN_GPIO_NUM;
    config_.pin_reset = RESET_GPIO_NUM;
    config_.xclk_freq_hz = 20000000;
    config_.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) 
    {
        config_.frame_size = FRAMESIZE_VGA;
        config_.jpeg_quality = 10;
        config_.fb_count = 2;
    } 
    else 
    {
        config_.frame_size = FRAMESIZE_HVGA;
        config_.jpeg_quality = 12;
        config_.fb_count = 1;
    }

    // Camera init
    init_err_= esp_camera_init(&config_);
    
}