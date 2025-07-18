#include "monicaWifiSettingModule.h"

MonicaWifiSettingModule::MonicaWifiSettingModule(MonicaConfig* config, MonicaWifi* wifi, MonicaWifiAp* esp_wifi)
{
    config_ = config;
    wifi_ = wifi;
    esp_wifi_ = esp_wifi;
}

MonicaWifiSettingModule::~MonicaWifiSettingModule()
{
}

void MonicaWifiSettingModule::initilize()
{
    config_->initialize();



    String ssid = config_->get_config("ssid");
    String password = config_->get_config("password");
    wifi_->set_ssid(ssid);
    wifi_->set_password(password);

    String esp_ssid = config_->get_config("esp_ssid");
    String esp_password = config_->get_config("esp_password");
    esp_wifi_->set_ssid(esp_ssid);
    esp_wifi_->set_password(esp_password);


    
    wifi_->initialize();
    esp_wifi_->initialize();
}
