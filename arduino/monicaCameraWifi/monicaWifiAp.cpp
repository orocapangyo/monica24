#include "monicaWifiAp.h"

MonicaWifiAp::MonicaWifiAp()
{
    is_first_initialized_ = true;
}

MonicaWifiAp::MonicaWifiAp(char *esp_ssid, char *esp_password)
{
    set_ssid(esp_ssid);
    set_password(esp_password);

    is_first_initialized_ = true;
}


void MonicaWifiAp::initialize()
{
    if (is_first_initialized_ == false)
    {
        return;
    }

    is_first_initialized_ = false;

    

    WiFi.mode(WIFI_AP_STA);
    if (!strcmp(esp_password_, ""))
    {
        WiFi.softAP(esp_ssid_);
    }
    else
    {
        WiFi.softAP(esp_ssid_, esp_password_);
    }

    esp_ip_ = WiFi.softAPIP().toString();
}