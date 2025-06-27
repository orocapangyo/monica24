
#include "MotorModule.h"
#include <Arduino.h>

MonicaMotor* MotorModule::motor_left_ = nullptr;
MonicaMotor* MotorModule::motor_right_ = nullptr;

MotorModule::MotorModule( MonicaMotor* motor_left,
                          MonicaMotor* motor_right)
{
  set_motor_left(motor_left);
  set_motor_right(motor_right);
}


MotorModule::~MotorModule()
{

}


void MotorModule::initialize()
{
  int enc_in_left_pin = motor_left_->get_enc_in_pin_a();
  int enc_in_right_pin = motor_right_->get_enc_in_pin_a();

  pinMode(enc_in_left_pin, INPUT);
  pinMode(enc_in_left_pin, INPUT_PULLUP);

  pinMode(enc_in_right_pin, INPUT);
  pinMode(enc_in_right_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc_in_left_pin), motor_left_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_in_right_pin), motor_right_encoder_interrupt, RISING);
}

void MotorModule::motor_left_encoder_interrupt()
{
  if (motor_left_)
  {
    motor_left_->wheel_tick();
  }
}

void MotorModule::motor_right_encoder_interrupt()
{
  if (motor_right_)
  {
    motor_right_->wheel_tick();
  }
}
