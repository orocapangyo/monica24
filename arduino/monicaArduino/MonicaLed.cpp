#include <Arduino.h>
#include "MonicaLed.h"


MonicaLed::MonicaLed(int led_left_pin,
                     int led_right_pin,
                     int led_front_pin)
{
  led_left_pin_ = led_left_pin;
  led_right_pin_ = led_right_pin;
  led_front_pin_ = led_front_pin;
}

MonicaLed::~MonicaLed()
{
}


void MonicaLed::Initialize()
{
  pinMode(led_left_pin_, OUTPUT);
  pinMode(led_right_pin_, OUTPUT);
  pinMode(led_front_pin_, OUTPUT);
  RGB(LED_ALL_OFF);  // RGB LED all off

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);
}


void MonicaLed::RGB(int led_active)
{
  switch (led_active) 
  {
  case LED_ALL_OFF:
    digitalWrite(led_right_pin_, LOW);
    digitalWrite(led_left_pin_, LOW);
    digitalWrite(led_front_pin_, LOW);
    break;
  case LED_FRONT_ON:
    digitalWrite(led_right_pin_, LOW);
    digitalWrite(led_left_pin_, LOW);
    digitalWrite(led_front_pin_, HIGH);
    break;
  case LED_LEFT_ON:
    digitalWrite(led_right_pin_, LOW);
    digitalWrite(led_left_pin_, HIGH);
    digitalWrite(led_front_pin_, LOW);
    break;
  case LED_RIGHT_ON:
    digitalWrite(led_right_pin_, HIGH);
    digitalWrite(led_left_pin_, LOW);
    digitalWrite(led_front_pin_, LOW);
    break;
  case LED_REAR_ON:
    digitalWrite(led_right_pin_, HIGH);
    digitalWrite(led_left_pin_, HIGH);
    digitalWrite(led_front_pin_, LOW);
    break;
  case LED_ALL_ON:
    digitalWrite(led_right_pin_, HIGH);
    digitalWrite(led_left_pin_, HIGH);
    digitalWrite(led_front_pin_, HIGH);
    break;
  default:
    digitalWrite(led_right_pin_, LOW);
    digitalWrite(led_left_pin_, LOW);
    digitalWrite(led_front_pin_, LOW);
    break;
  }
}