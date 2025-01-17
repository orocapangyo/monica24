#ifndef _MONICA_MPU_6050_H_
#define _MONICA_MPU_6050_H_

#include "MPU6050_6Axis_MotionApps20.h"

class MonicaMPU6050
{
public:
    MonicaMPU6050(uint8_t mpu_pin);
    ~MonicaMPU6050();

public:
    void initialize();
    void check_value();

public:
    void set_mpu_interrupt(bool mpu_interrupt) { mpuInterrupt = mpu_interrupt; }
    void set_dmp_ready(bool dmp_ready) { dmpReady = dmp_ready; }
    
    uint8_t get_dev_status() { return devStatus; }
    uint8_t get_mpu_pin() { return mpu_pin_; }
    Quaternion* get_quaternion() {return &q; }
    

public:
    void dmp_set_enabled() { mpu.setDMPEnabled(true); }
    void check_to_get_int_status() { mpuIntStatus = mpu.getIntStatus(); }
    void dmp_get_fifo_packet_size() { packetSize = mpu.dmpGetFIFOPacketSize(); }

private:
    MPU6050 mpu;
    uint8_t mpu_pin_;

    bool is_initialize_;
    volatile bool mpuInterrupt;  // indicates whether MPU interrupt pin has gone high
    // MPU control/status vars
    bool dmpReady;   // set true if DMP init was successful
    uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
    uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;      // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];  // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;  // [w, x, y, z]         quaternion container
};
#endif /*_MONICA_MPU_6050_H_*/
