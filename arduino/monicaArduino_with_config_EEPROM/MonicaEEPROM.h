#ifndef _MONICA_EEPROM_H_
#define _MONICA_EEPROM_H_

#include <Arduino.h>

class MonicaEEPROM
{
public:
    MonicaEEPROM();
    ~MonicaEEPROM();

public:
    void initialize();
    void begin();
    void end();
    bool is_enable_to_use();

public:
    void save_used_wifi(bool is_wifi);
    void save_domain_id(int domain_id);
    void save_ssid(char* ssid);
    void save_password(char* password);
    void save_agent_ip(char* agent_ip);
    void save_port(int port);

    bool load_used_wifi();
    int load_domain_id();
    String load_ssid(char* copy_ssid = nullptr);
    String load_password(char* copy_password = nullptr);
    String load_agent_ip(char* copy_agent_ip = nullptr);
    int load_port();


private:
    bool is_first_initialized_;

};
#endif /*_MONICA_EEPROM_H_*/

