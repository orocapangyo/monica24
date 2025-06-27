#ifndef _MONICA_MPU_MODULE_H_
#define _MONICA_MPU_MODULE_H_

#include "MonicaMPU6050.h"

class MonicaMPUModule
{
public:
    MonicaMPUModule(MonicaMPU6050* mpu);
    ~MonicaMPUModule();

public:
    static void initialize();


public:
    void set_mpu(MonicaMPU6050* mpu) { mpu_ = mpu; }

private:
    static void mpu_dmp_ready_interrupt();

private:
    static MonicaMPU6050* mpu_;

};
#endif /*_MONICA_MPU_MODULE_H_*/
