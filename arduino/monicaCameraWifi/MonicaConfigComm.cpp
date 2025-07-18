#include <Arduino.h>
#include "MonicaConfigComm.h"

#define __HEAD          0xFF
#define __DEVICE_ID     0xF8
#define __RETURN_ID     0xF7
#define __READ_DATA     0x50

#define TIMEOUT_MILLISEC     5000
#define WAIT_TIMEOUT_MILLISEC   8000

enum
{
    ORDER_WIFI_SSID = 0x01,
    ORDER_WIFI_PASSWD = 0x02,
    ORDER_AGENT_IP = 0x03,
    ORDER_AGENT_PORT = 0x04,
    ORDER_CAR_TYPE = 0x05,
    ORDER_DOMAIN_ID = 0x06,
    ORDER_ESP_SSID = 0x0C,
    ORDER_ESP_PASSWORD = 0x0D,
    ORDER_REQUEST = __READ_DATA,
    ORDER_END = 0xff
};


enum
{
    STATE_READ,
    STATE_PARSING,
    STATE_CLEAR
};

enum
{
    CHECK_STEP_HEAD,
    CHECK_STEP_DEVICE_ID,
    CHECK_STEP_DATA_LENGTH,
    CHECK_STEP_DATA_END
};

//MonicaConfigComm::MonicaConfigComm(MonicaEEPROM* eeprom_module)
//{
//    eeprom_module_ = eeprom_module;
//    clear();
//    is_receive_config_ = false;
//    start_wait_time_ = 0;
//    last_receive_time_ = 0;
//}

MonicaConfigComm::MonicaConfigComm(MonicaConfig* config)
{
    config_ = config;
    clear();
    is_receive_config_ = false;
    start_wait_time_ = 0;
    last_receive_time_ = 0;
}

MonicaConfigComm::~MonicaConfigComm()
{

}

void MonicaConfigComm::initialize()
{
    Serial.begin(115200);   // don't change baurdrate 115200, because it is same baurdrate micor_ros_serial
}

void MonicaConfigComm::wait_to_receive()
{
    start_wait_time_ = millis();

    while (is_wait_time() || Serial.available())
    {
        receive();
    }
}

void MonicaConfigComm::receive()
{
    switch (state_)
    {
        case STATE_READ:
            read();
            break;

        case STATE_PARSING:
            parsing();
            break;

        case STATE_CLEAR:
            clear();
            break;
        
        default:
            // Error!
            break;
    }

    //check_timeout();
}

void MonicaConfigComm::read()
{
    if (Serial.available() > 0)
    {
        buffer_[index_] = Serial.read();
    #ifdef DEBUG_CONFIG_2
        Serial.write(buffer_[index_]);
    #endif

        check();
        index_++;
    }
}


void MonicaConfigComm::check()
{
    switch(check_step_)
    {
        case CHECK_STEP_HEAD:          check_head();        break;    
        case CHECK_STEP_DEVICE_ID:     check_device_id();   break; 
        case CHECK_STEP_DATA_LENGTH:   check_data_length(); break;
        case CHECK_STEP_DATA_END:      check_data_end();    break;   
    }
}

void MonicaConfigComm::check_head()
{
    if (check_step_ != CHECK_STEP_HEAD)
        return;

    if (buffer_[index_] == __HEAD)
    {
        check_step_ = CHECK_STEP_DEVICE_ID;
        head_index_ = index_;
    }
#ifdef DEBUG_CONFIG_2
    Serial.print("check_head ");
    Serial.println(buffer_[index_], HEX);
#endif
}

void MonicaConfigComm::check_device_id()
{
    if (check_step_ != CHECK_STEP_DEVICE_ID)
        return;
    
    if (buffer_[index_] == __DEVICE_ID)
    {
        check_step_ = CHECK_STEP_DATA_LENGTH;
    #ifdef DEBUG_CONFIG_2
        Serial.print("check_device_id ");
        Serial.println(buffer_[index_], HEX);
    #endif
    }
    else
    {
        // go back to check head
        check_step_ = CHECK_STEP_HEAD;
        head_index_ = 0;
    #ifdef DEBUG_CONFIG_2
        Serial.print("check_device_id fail ");
        Serial.println(buffer_[index_], HEX);
    #endif
    }
}

void MonicaConfigComm::check_data_length()
{
    if (check_step_ != CHECK_STEP_DATA_LENGTH)
        return;
    
    data_length_ = buffer_[index_];
    remain_data_ = data_length_ - 3; // for no check head, device_id, data_length 
    check_step_ = CHECK_STEP_DATA_END;

#ifdef DEBUG_CONFIG_2
    Serial.println("check_data_length");
#endif
}

void MonicaConfigComm::check_data_end()
{
    if (check_step_ != CHECK_STEP_DATA_END)
        return;
    
    if (remain_data_ == 1)
    {
        int check_sum = 0;
        for (uint8_t idx = 0; idx < (data_length_-1); idx++)
        {
            check_sum += buffer_[head_index_+idx];
        }
        check_sum = check_sum % 256;

        sum_data_ = buffer_[index_];
        if (check_sum == sum_data_)
        {
            state_ = STATE_PARSING;
            is_receive_config_ = true;
            last_receive_time_ = millis();
        #ifdef DEBUG_CONFIG_2
            Serial.print("check_data_end success ");
            //Serial.print("check_sum=");
            //Serial.print(check_sum, HEX);
            //Serial.print(" sum_data_=");
            //Serial.println(sum_data_, HEX);
        #endif
        }
        else
        {
            state_ = STATE_CLEAR;
        #ifdef DEBUG_CONFIG_2
            Serial.print("check_data_end fail ");
            //Serial.print("check_sum=");
            //Serial.print(check_sum, HEX);
            //Serial.print(" sum_data_=");
            //Serial.println(sum_data_, HEX);
        #endif
        }
    }
    else
    {
        remain_data_--;
    }
}

void MonicaConfigComm::check_timeout()
{
    if (is_receive_config_)
    {
        if ((millis() - last_receive_time_) >= TIMEOUT_MILLISEC)
        {
            //Serial.println("restart");
            ESP.restart();
            return;
        }
        //Serial.print("time: ");
        //Serial.println(millis() - last_receive_time_);
    }
}

void MonicaConfigComm::parsing()
{
    // Receive Data:
    // [__HEAD, __DEVICE_ID, value_length+5, order, values, sum_data]
    order_ = buffer_[head_index_+3];
    value_length_ = data_length_-5;
#ifdef DEBUG_CONFIG_2
    Serial.println("parsing");
#endif
    update();
    state_ = STATE_CLEAR;
}

void MonicaConfigComm::update()
{
    char* value = &(buffer_[head_index_+4]);
    
    //eeprom_module_->begin();
    switch (order_)
    {
        case ORDER_WIFI_SSID:
        {
            char ssid[64];
            memset(ssid, 0, sizeof(char)*64);
            memcpy(ssid, value, (sizeof(char)*value_length_));

            //eeprom_module_->save_ssid(ssid);
            config_->set_config("ssid" ,String(ssid));
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_WIFI_SSID ");
            Serial.print(value);
            Serial.print(", ");
            Serial.print(ssid);
            Serial.print(", length: ");
            Serial.print(value_length_);
            Serial.println("---");
        #endif
        }
        break;
        
        case ORDER_WIFI_PASSWD:
        {
            char password[32];
            memset(password, 0, sizeof(char)*32);
            memcpy(password, value, (sizeof(char)*value_length_));
            //eeprom_module_->save_password(password);
            config_->set_config("password" ,String(password));
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_WIFI_PASSWD ");
            Serial.println(password);
        #endif
        }
        break;

        case ORDER_ESP_SSID:
        {
            char esp_ssid[64];
            memset(esp_ssid, 0, sizeof(char)*64);
            memcpy(esp_ssid, value, (sizeof(char)*value_length_));

            config_->set_config("esp_ssid" ,String(esp_ssid));
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_ESP_SSID ");
            Serial.print(value);
            Serial.print(", ");
            Serial.print(esp_ssid);
            Serial.print(", length: ");
            Serial.print(value_length_);
            Serial.println("---");
        #endif
        }
        break;
        
        case ORDER_ESP_PASSWORD:
        {
            char esp_password[32];
            memset(esp_password, 0, sizeof(char)*32);
            memcpy(esp_password, value, (sizeof(char)*value_length_));

            config_->set_config("esp_password" ,String(esp_password));
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_WIFI_PASSWD ");
            Serial.println(esp_password);
        #endif
        }
        break;
        
        /*
        case ORDER_AGENT_IP:
        {
            String agent_ip_str;
            agent_ip_str = String((int)value[0]) 
                         + "." 
                         + String((int)value[1])
                         + "." 
                         + String((int)value[2]) 
                         + "." 
                         + String((int)value[3]);
            char agent_ip[32];
            memset(agent_ip, 0, sizeof(char)*32);
            agent_ip_str.toCharArray(agent_ip, sizeof(agent_ip));
            eeprom_module_->save_agent_ip(agent_ip);
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_AGENT_IP ");
            Serial.print(agent_ip_str);
            Serial.print("---");
            Serial.println(agent_ip);
        #endif
        }
        break;

        case ORDER_AGENT_PORT:
        {
            int port = (int)(value[0] + ((int)(value[1])<<8));
            eeprom_module_->save_port(port);
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_AGENT_PORT ");
            Serial.println(port);
        #endif
        }
        break;

        case ORDER_CAR_TYPE:
        {
            bool is_wifi = (bool)(value[0] == 0);
            eeprom_module_->save_used_wifi(is_wifi);
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_CAR_TYPE ");
            Serial.println(is_wifi);
        #endif
        }
        break;

        case ORDER_DOMAIN_ID:
        {
            int domain_id = (int)(value[0] + ((int)(value[1])<<8));
            eeprom_module_->save_domain_id(domain_id);
        #ifdef DEBUG_CONFIG_2
            Serial.print("update ORDER_DOMAIN_ID ");
            Serial.println(domain_id);
        #endif
        }
        break;
        */

        default:
            break;
    }
    //eeprom_module_->end();

    if (order_ == ORDER_REQUEST)
    {
        response();
    }
}


void MonicaConfigComm::response()
{
    uint8_t request_order_addr = buffer_[head_index_+4];
    char* param = &(buffer_[head_index_+5]);
    char value[64];
    int value_length = 0;
    memset(value, 0, sizeof(char)*64);

    switch (request_order_addr)
    {
        case ORDER_WIFI_SSID:
        {
            //eeprom_module_->load_ssid(value);
            String ssid_str = config_->get_config("ssid");
            strcpy(value, ssid_str.c_str());
            value_length = ssid_str.length();
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_WIFI_SSID ");
            Serial.println(value);
        #endif
        }
        break;
        
        case ORDER_WIFI_PASSWD:
        {
            //eeprom_module_->load_password(value);
            String password_str = config_->get_config("password");
            strcpy(value, password_str.c_str());
            value_length = password_str.length();
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_WIFI_PASSWD ");
            Serial.println(value);
        #endif
        }
        break;

        case ORDER_ESP_SSID:
        {
            String esp_ssid_str = config_->get_config("esp_ssid");
            strcpy(value, esp_ssid_str.c_str());
            value_length = esp_ssid_str.length();
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_ESP_SSID ");
            Serial.println(value);
        #endif
        }
        break;
        
        case ORDER_ESP_PASSWORD:
        {
            String esp_password_str = config_->get_config("esp_password");
            strcpy(value, esp_password_str.c_str());
            value_length = esp_password_str.length();
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_ESP_PASSWORD ");
            Serial.println(value);
        #endif
        }
        break;
        
        /*
        case ORDER_AGENT_IP:
        {
            String agent_ip_str = eeprom_module_->load_agent_ip();
            int first = agent_ip_str.indexOf(".");
            int second = agent_ip_str.indexOf(".", first+1);
            int third = agent_ip_str.indexOf(".", second+1);
            int length = agent_ip_str.length();

            value[0] = (char)agent_ip_str.substring(0, first).toInt();
            value[1] = (char)agent_ip_str.substring(first+1, second).toInt();
            value[2] = (char)agent_ip_str.substring(second+1, third).toInt();
            value[3] = (char)agent_ip_str.substring(third+1, length).toInt();

            value_length = 4;
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_AGENT_IP ");
            //Serial.print((int)value[0]);
            //Serial.print((int)value[1]);
            //Serial.print((int)value[2]);
            Serial.println(value);
        #endif
        }
        break;

        case ORDER_AGENT_PORT:
        {
            int port = eeprom_module_->load_port();
            value[0] = (port & 0xFF);
            value[1] = ((port>>8) & 0xFF);
            value_length = 2;
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_AGENT_PORT ");
            Serial.println(port);
        #endif
            
        }
        break;

        case ORDER_CAR_TYPE:
        {
            bool is_wifi = eeprom_module_->load_used_wifi();
            value[0] = (is_wifi ? 0 : 1) ;
            value[1] = 0;
            value_length = 2;
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_CAR_TYPE ");
            Serial.println(is_wifi);
        #endif
        }
        break;

        case ORDER_DOMAIN_ID:
        {
            int domain_id = eeprom_module_->load_domain_id();
            value[0] = (domain_id & 0xFF);
            value[1] = ((domain_id>>8) & 0xFF);
            value_length = 2;
        #ifdef DEBUG_CONFIG_2
            Serial.print("response ORDER_DOMAIN_ID ");
            Serial.println(domain_id);
        #endif
        }
        break;
        */

        default:
            //break;
            return;
    }

    #ifdef DEBUG_CONFIG_2
        Serial.print("    ");
    #endif

    uint8_t response_packat[96];
    memset(response_packat, 0, sizeof(char)*96);
    uint32_t check_sum = 0;
    int index = 0;

    response_packat[0] = __HEAD;
    response_packat[1] = __RETURN_ID;
    response_packat[2] = value_length + 5;
    response_packat[3] = request_order_addr;
    for (index = 0; index < value_length; index++)
    {
        response_packat[index+4] = value[index];
        check_sum += value[index];
    }

    check_sum = (check_sum + __HEAD + __RETURN_ID + (value_length + 5) + request_order_addr) % 256;
    response_packat[index+4] = (uint8_t)check_sum;

    Serial.write(response_packat, (value_length + 5));
}

void MonicaConfigComm::clear()
{
    memset(buffer_, 0, sizeof(char)*MAX_BUFFER_SIZE);
    index_ = 0;
    state_ = STATE_READ;
    check_step_ = CHECK_STEP_HEAD;
    head_index_ = 0;

    order_ = 0;
    data_length_ = 0;
    remain_data_ = 0;
    value_length_ = 0;
    sum_data_ = 0;
}

bool MonicaConfigComm::is_wait_time()
{
    return ((millis() - start_wait_time_) < WAIT_TIMEOUT_MILLISEC);
}
    




