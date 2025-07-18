#ifndef _MONICA_WIFI_SETTING_MODULE_H_
#define _MONICA_WIFI_SETTING_MODULE_H_

#include "monicaConfig.h"
#include "monicaWifi.h"
#include "monicaWifiAp.h"


class MonicaWifiSettingModule
{
public:
    MonicaWifiSettingModule(MonicaConfig* config, MonicaWifi* wifi, MonicaWifiAp* esp_wifi);
    ~MonicaWifiSettingModule();

public:
    void initilize();

private:
    MonicaConfig* config_;
    MonicaWifi* wifi_;
    MonicaWifiAp* esp_wifi_;
};


#endif /*_MONICA_WIFI_SETTING_MODULE_H_*/
