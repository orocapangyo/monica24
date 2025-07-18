#ifndef _MONICA_WIFI_AP_H_
#define _MONICA_WIFI_AP_H_

#include <WiFi.h>
#include <WiFiAP.h>
#include <Arduino.h>

#define _MAX_LEN (64)

class MonicaWifiAp
{
public:
    MonicaWifiAp();
    MonicaWifiAp(char *esp_ssid, char *esp_password);

public:
    void initialize();

public:
    void set_ssid(char *ssid) { memcpy(esp_ssid_, ssid, sizeof(ssid)); }
    void set_ssid(String ssid_str) { strcpy(esp_ssid_, ssid_str.c_str()); }
    void set_password(char *password) { memcpy(esp_password_, password, sizeof(password)); }
    void set_password(String password_str) { strcpy(esp_password_, password_str.c_str()); }
    int get_wifi_state();
    String get_esp_ip() { return esp_ip_; }

private:
    char esp_ssid_[_MAX_LEN];
    char esp_password_[_MAX_LEN];
    String esp_ip_;

    bool is_first_initialized_;

};

#endif /*_MONICA_WIFI_AP_H_*/
