#ifndef _MONICA_COMM_MODULE_
#define _MONICA_COMM_MODULE_

#include "MonicaConfigComm.h"
#include "MonicaRosComm.h"
#include "MonicaEEPROM.h"


class MonicaCommModule
{
public:
    MonicaCommModule(MonicaConfigComm* config_comm, MonicaRosComm* ros_comm, MonicaEEPROM* eeprom_module);
    ~MonicaCommModule();

public:
    void initialize();

private:
    static bool monica_arduino_transport_open(struct uxrCustomTransport * transport);
    static bool monica_arduino_transport_close(struct uxrCustomTransport * transport);
    static size_t monica_arduino_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
    static size_t monica_arduino_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

private:
    static MonicaConfigComm* config_comm_;
    static MonicaRosComm* ros_comm_;
    static MonicaEEPROM* eeprom_module_;

};
#endif /*_MONICA_COMM_MODULE_*/
