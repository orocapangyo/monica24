#ifndef _MONICA_WIFI_H_
#define _MONICA_WIFI_H_

#include <WiFi.h>
#include <Arduino.h>

#define MAX_LEN (64)

class MonicaWifi
{
public:
    MonicaWifi();
    MonicaWifi(char *ssid, char *password);

public:
    void initialize();


public:
    void set_ssid(char *ssid) { memcpy(ssid_, ssid, sizeof(ssid)); }
    void set_ssid(String ssid_str) { strcpy(ssid_, ssid_str.c_str()); }
    void set_password(char *password) { memcpy(password_, password, sizeof(password)); }
    void set_password(String password_str) { strcpy(password_, password_str.c_str()); }
    int get_wifi_state();
    String get_local_ip() { return local_ip_; }

private:
    char ssid_[MAX_LEN];
    char password_[MAX_LEN];
    String local_ip_;
    int count_;

    bool is_first_initialized_;
};

#endif /*_MONICA_WIFI_H_*/
