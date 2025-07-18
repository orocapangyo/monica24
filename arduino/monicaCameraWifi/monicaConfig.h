#ifndef _MONICA_CONFIG_H_
#define _MONICA_CONFIG_H_


#include <Arduino.h>
#include <Preferences.h>



#define CONFIG_ESP_SSID         "ESP_WIFI"
#define CONFIG_ESP_PASSWORD     ""

class MonicaConfig
{
public:
    MonicaConfig();
    ~MonicaConfig();

public:
    void initialize();

    void set_config(String key, String value);
    String get_config(String key, String default_value = "");

private:
    bool is_first_startup();

    
private:
    Preferences preferences_;
    bool is_first_initialized_;
};

#endif /*_MONICA_CONFIG_H_*/
