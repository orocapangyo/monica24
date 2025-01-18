#include <Arduino.h>
#include "MonicaMPUModule.h"

MonicaMPU6050* MonicaMPUModule::mpu_ = nullptr;

MonicaMPUModule::MonicaMPUModule(MonicaMPU6050* mpu)
{
  set_mpu(mpu);
}

MonicaMPUModule::~MonicaMPUModule()
{
}


void MonicaMPUModule::initialize()
{
  if (mpu_ != nullptr) 
  {
    return;
  }


  uint8_t mup_pin = mpu_->get_mpu_pin();

  pinMode(mup_pin, INPUT);

  mpu_->initialize();

  if (mpu_->get_dev_status() == 0)
  {
    mpu_->dmp_set_enabled();

    attachInterrupt(digitalPinToInterrupt(mup_pin), mpu_dmp_ready_interrupt, RISING);
    mpu_->check_to_get_int_status();

    mpu_->set_dmp_ready(true);
    mpu_->dmp_get_fifo_packet_size();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //DEBUG_PRINT(F("DMP Initialization failed (code "));
    //DEBUG_PRINT(devStatus);
    //DEBUG_PRINTLN(F(")"));
  }
}


void MonicaMPUModule::mpu_dmp_ready_interrupt()
{
    if (mpu_ != nullptr)
    {
        mpu_->set_dmp_ready(true);
    }
}
