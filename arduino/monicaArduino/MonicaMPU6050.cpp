#include <Arduino.h>
#include "MonicaMPU6050.h"

#include "I2Cdev.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MonicaMPU6050::MonicaMPU6050(uint8_t mpu_pin)
{
  mpu_pin_ = mpu_pin;
  is_initialize_ = false;
  mpuInterrupt = false;
  dmpReady = false;
  mpuIntStatus = 1;
  devStatus = 1;
  packetSize = 0;
  fifoCount = 0;
  memset(fifoBuffer, 0, sizeof(fifoBuffer));
}

MonicaMPU6050::~MonicaMPU6050()
{

}

void MonicaMPU6050::initialize()
{
  if (is_initialize_) return;

  is_initialize_ = true; 

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();
  //pinMode(mpu_pin_, INPUT);

  // verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(103);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(595);  // 1688 factory default for my test chip

#if 0
  if (devStatus == 0) 
  {
    // turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else 
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(devStatus);
    DEBUG_PRINTLN(F(")"));
  }
#endif
}


void MonicaMPU6050::check_value()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet
  {
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
#if (PRINT_QUAT == 1)
    DEBUG_PRINT("quat\t");
    DEBUG_PRINT(q.w);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.x);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(q.y);
    DEBUG_PRINT("\t");
    DEBUG_PRINTLN(q.z);
#endif
  }
}
