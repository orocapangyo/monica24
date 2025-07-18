#include "Arduino.h"
#include "monicaConfig.h"
#include "MonicaConfigComm.h"
#include "monicaCamera.h"
#include "monicaWifiAp.h"
#include "monicaWifi.h"
#include "monicaHttp.h"
#include "monicaWifiSettingModule.h"
#include "camera_pinmap.h"

MonicaConfig config;
MonicaConfigComm config_comm(&config);

MonicaCamera camera;
MonicaWifiAp esp_wifi;
MonicaWifi wifi;
MonicaHttp http;

MonicaWifiSettingModule wifi_setting_module(&config, &wifi, &esp_wifi);



void setup()
{
    // must be call first
    wifi_setting_module.initilize();

    config.initialize();
    config_comm.initialize();
    camera.initialize();
    esp_wifi.initialize();
    wifi.initialize();
    http.initialize();

    Serial.begin(115200);
    Serial.setDebugOutput(false);

    Serial.print("ESP SSID: ");
    Serial.println(config.get_config("esp_ssid"));
    Serial.print("ESP PASSWORD: ");
    Serial.println(config.get_config("esp_password"));
    Serial.print("ESP IP: ");
    Serial.println(esp_wifi.get_esp_ip());


    Serial.print("WiFi SSID: ");
    Serial.println(config.get_config("ssid"));
    Serial.print("WiFi PASSWORD: ");
    Serial.println(config.get_config("password"));
    Serial.print("WiFi IP: ");
    Serial.println(wifi.get_local_ip());
}


void loop()
{
    // camera and wifi process run other threads
    config_comm.receive();
    config_comm.check_timeout();
}