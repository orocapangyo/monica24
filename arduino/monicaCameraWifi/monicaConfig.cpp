#include "monicaConfig.h"

#define CONFIG_NAME_NAMESPACE   "monica_config"


MonicaConfig::MonicaConfig()
{
    is_first_initialized_ = true;
}

MonicaConfig::~MonicaConfig()
{

}

void MonicaConfig::initialize()
{
    if (is_first_initialized_ == false)
    {
        return;
    }

    is_first_initialized_ = false;




    preferences_.begin(CONFIG_NAME_NAMESPACE);
    if (is_first_startup())
    {
        preferences_.putString("ssid", "");
        preferences_.putString("password", "");
        preferences_.putString("esp_ssid", CONFIG_ESP_SSID);
        preferences_.putString("esp_password", CONFIG_ESP_PASSWORD);
        
        preferences_.putBool("first_startup", false);
    }
}

bool MonicaConfig::is_first_startup()
{
    return preferences_.getBool("first_startup", true);
}

void MonicaConfig::set_config(String key, String value)
{
    if (key.length() > 0)
    {
        preferences_.putString(key.c_str(), value.c_str());
    }
}

String MonicaConfig::get_config(String key, String default_value)
{
    return preferences_.getString(key.c_str(), default_value.c_str());
}

