#ifndef _MONICA_CONFIG_H_
#define _MONICA_CONFIG_H_

#include "MonicaEEPROM.h"

#define MAX_BUFFER_SIZE 256
#define DEBUG_CONFIGxx
#define DEBUG_CONFIG_2xx

class MonicaConfigComm
{
public:
    MonicaConfigComm(MonicaEEPROM* eeprom_module);
    ~MonicaConfigComm();

public:
    void initialize();
    void wait_to_receive();
    void receive();
    void check_timeout();
    bool is_receive_config() { return is_receive_config_; }
    bool is_wait_time();

private:
    void read();
    void check();
    void check_head();
    void check_device_id();
    void check_data_length();
    void check_data_end();
    void parsing();
    void update();
    void response();
    void clear();

private:
    char buffer_[MAX_BUFFER_SIZE];
    uint8_t index_;
    uint8_t state_;
    uint8_t check_step_;
    uint8_t head_index_;

    uint8_t order_;
    uint8_t data_length_;
    uint8_t remain_data_;
    uint8_t value_length_;
    int sum_data_;

    bool is_receive_config_;
    unsigned long start_wait_time_;
    unsigned long last_receive_time_;

    MonicaEEPROM* eeprom_module_;

};
#endif /*_MONICA_CONFIG_H_*/

