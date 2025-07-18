#include "monicaWifi.h"

MonicaWifi::MonicaWifi()
{
    count_ = 0;
    is_first_initialized_ = true;
}

MonicaWifi::MonicaWifi(char *ssid, char *password)
{
    set_ssid(ssid);
    set_password(password);

    count_ = 0;
    is_first_initialized_ = true;
}


void MonicaWifi::initialize()
{
    if (is_first_initialized_ == false)
    {
        return;
    }

    is_first_initialized_ = false;



    WiFi.begin(ssid_, password_);
    WiFi.setSleep(false);

    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        count_++;
        if (count_ > 3) // 1.5 seconds timeout
        {
            return;
        }
    }

    local_ip_ = WiFi.localIP().toString();
}

int MonicaWifi::get_wifi_state()
{
    return (int)WiFi.status();
}

