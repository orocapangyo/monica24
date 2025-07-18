#ifndef _MONICA_CAMERA_H_
#define _MONICA_CAMERA_H_

#include <esp_camera.h>

class MonicaCamera
{
public:
    MonicaCamera();

public:
    void initialize();

public:
    esp_err_t get_init_error_code() { return init_err_; }

private:
    camera_config_t config_;
    esp_err_t init_err_;
};

#endif /*_MONICA_CAMERA_H_*/
