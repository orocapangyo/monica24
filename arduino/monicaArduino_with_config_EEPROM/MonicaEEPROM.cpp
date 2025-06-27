#include <EEPROM.h>
#include "PinMap.h"
#include "MonicaEEPROM.h"

#define EEPROM_SIZE 1024

enum
{
    EEPROM_ADDRESS_PRODUCT_ID = 0,
    EEPROM_ADDRESS_USED_WIFI = 0x01,
    EEPROM_ADDRESS_DOMAIN_ID = 0x02,
    EEPROM_ADDRESS_SSID = 0x10,
    EEPROM_ADDRESS_PASSWORD = 0x110,
    EEPROM_ADDRESS_AGENT_IP = 0x200,
    EEPROM_ADDRESS_PORT = 0x300,
    EEPROM_ADDRESS_END = 0x2000
};


MonicaEEPROM::MonicaEEPROM()
{
    is_first_initialized_ = false;
}

MonicaEEPROM::~MonicaEEPROM()
{

}

void MonicaEEPROM::initialize()
{
    if (is_first_initialized_)
        return;

    is_first_initialized_ = true;

    
    EEPROM.begin(EEPROM_SIZE);
}


void MonicaEEPROM::begin()
{
    if (EEPROM.readByte(EEPROM_ADDRESS_PRODUCT_ID) != MONICA_PRODUCT_ID)
    {
        EEPROM.writeByte(EEPROM_ADDRESS_PRODUCT_ID, MONICA_PRODUCT_ID);
    }
}

void MonicaEEPROM::save_used_wifi(bool is_wifi)
{
    EEPROM.writeBool(EEPROM_ADDRESS_USED_WIFI, is_wifi);
}

void MonicaEEPROM::save_domain_id(int domain_id)
{
    EEPROM.writeInt(EEPROM_ADDRESS_DOMAIN_ID, domain_id);
}

void MonicaEEPROM::save_ssid(char* ssid)
{
    EEPROM.writeString(EEPROM_ADDRESS_SSID, ssid);
}

void MonicaEEPROM::save_password(char* password)
{
    EEPROM.writeString(EEPROM_ADDRESS_PASSWORD, password);
}

void MonicaEEPROM::save_agent_ip(char* agent_ip)
{
    EEPROM.writeString(EEPROM_ADDRESS_AGENT_IP, agent_ip);
}

void MonicaEEPROM::save_port(int port)
{
    EEPROM.writeInt(EEPROM_ADDRESS_PORT, port);
}

bool MonicaEEPROM::load_used_wifi()
{
    return EEPROM.readBool(EEPROM_ADDRESS_USED_WIFI);
}

int MonicaEEPROM::load_domain_id()
{
    return EEPROM.readInt(EEPROM_ADDRESS_DOMAIN_ID);
}

String MonicaEEPROM::load_ssid(char* copy_ssid)
{
    String ori_ssid = EEPROM.readString(EEPROM_ADDRESS_SSID);
    if (copy_ssid != nullptr)
    {
        strcpy(copy_ssid, ori_ssid.c_str());
    }
    return ori_ssid;
}

String MonicaEEPROM::load_password(char* copy_password)
{
    String ori_password = EEPROM.readString(EEPROM_ADDRESS_PASSWORD);
    if (copy_password != nullptr)
    {
        strcpy(copy_password, ori_password.c_str());
    }
    return ori_password;
}

String MonicaEEPROM::load_agent_ip(char* copy_agent_ip)
{
    String ori_agent_ip = EEPROM.readString(EEPROM_ADDRESS_AGENT_IP);
    if (copy_agent_ip != nullptr)
    {
        strcpy(copy_agent_ip, ori_agent_ip.c_str());
    }
    return ori_agent_ip;
}

int MonicaEEPROM::load_port()
{
    return EEPROM.readInt(EEPROM_ADDRESS_PORT);
}

void MonicaEEPROM::end()
{
    EEPROM.commit(); 
    //commit을 하게 되면 실제로 EEPROM에 저장합니다.
    //출처: https://answerofgod.tistory.com/529 [The Answer's Engineering Blog:티스토리]
}

bool MonicaEEPROM::is_enable_to_use()
{
    return (EEPROM.readByte(EEPROM_ADDRESS_PRODUCT_ID) == MONICA_PRODUCT_ID);
}
