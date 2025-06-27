
#include "MonicaMotor.h"
#include <Arduino.h>

// Minumum and maximum values for 32-bit integers
#define ENCODER_MINIMUM -2147483648
#define ENCODER_MAXIMUM 2147483647


#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)

#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.160)


#define Kp  (double)0.1
#define Ki  (double)2.0
#define Kd  (double)0.0



MonicaMotor::MonicaMotor( int _enc_in_pin_a, 
                          int _enc_in_pin_b,
                          int _forward_pin, 
                          int _reverse_pin,
                          int _standby_pin,
                          int _pwm_pin,
                          int _pwm_channel)
{
  enc_in_pin_a = _enc_in_pin_a;
  enc_in_pin_b = _enc_in_pin_b;
  direction_forward_pin = _forward_pin;
  direction_reverse_pin = _reverse_pin;
  standby_pin = _standby_pin;
  pwm_pin = _pwm_pin;
  pwm_channel = _pwm_channel;

  direction = true;
  wheel_tick_count = 0;
  prev_tick_count = 0;
  prev_time = 0;

  wheel_vel = 0;
  req_vel = 0;

  pwm_req = 0;
  pwm_out = 0;

  trackAdjustValue = 0.0;
  trackSetpoint = 0.0;
  trackError = 0.0;

  trackPID = new PID(&trackError, &trackAdjustValue, &trackSetpoint, Kp, Ki, Kd, DIRECT);
}

MonicaMotor::~MonicaMotor()
{
  if (trackPID != nullptr)
  {
    delete trackPID;
    trackPID = nullptr;
  }
}


void MonicaMotor::initialize()
{
  pinMode(direction_forward_pin, OUTPUT);
  pinMode(direction_reverse_pin, OUTPUT);
  pinMode(standby_pin, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(direction_forward_pin, LOW);
  digitalWrite(direction_reverse_pin, LOW);
  digitalWrite(standby_pin, HIGH);

  ledcSetup(pwm_channel, 1000, 8);  //1000Hz, 8bits = 256(0 ~ 255)
  ledcAttachPin(pwm_pin, pwm_channel);

  // Set the motor speed
  ledcWrite(pwm_channel, 0);

  // Set pin states of the encoder
  
  pinMode(enc_in_pin_b, INPUT);
  pinMode(enc_in_pin_b, INPUT_PULLUP);


  trackPID->SetMode(AUTOMATIC);
  trackPID->SetSampleTime(200);
  trackPID->SetOutputLimits(-20, 20);
}



// Increment the number of ticks
void MonicaMotor::wheel_tick() 
{
  // Read the value for the encoder for the left wheel
  int val = digitalRead(enc_in_pin_b);

  if (val == LOW) {
    direction = true;  // Reverse
  } else {
    direction = false;  // Forward
  }

  if (direction) {
    if (wheel_tick_count == ENCODER_MAXIMUM) {
      wheel_tick_count = ENCODER_MINIMUM;
    } else {
      wheel_tick_count++;
    }
  } else {
    if (wheel_tick_count == ENCODER_MINIMUM) {
      wheel_tick_count= ENCODER_MAXIMUM;
    } else {
      wheel_tick_count--;
    }
  }
  //msg_left_tick_count.data = wheel_tick_count;
}


void MonicaMotor::calc_vel() 
{
  int numOfTicks;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  numOfTicks = wheel_tick_count - prev_tick_count;

  // Calculate wheel velocity in meters per second
  wheel_vel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prev_time);
  // Keep track of the previous tick count
  prev_tick_count = wheel_tick_count;

  // Update the timestamp
  prev_time = (millis() / 1000.0);
}


void MonicaMotor::set_pwm_values() 
{
  int pwm_inc;

  // Set the direction of the motors
  if (pwm_req > 0) {  // Left wheel forward
    digitalWrite(direction_forward_pin, HIGH);
    digitalWrite(direction_reverse_pin, LOW);
  } else if (pwm_req < 0) {  // Left wheel reverse
    digitalWrite(direction_forward_pin, LOW);
    digitalWrite(direction_reverse_pin, HIGH);
  } else if (pwm_req == 0 && pwm_out == 0) {  // Left wheel stop
    digitalWrite(direction_forward_pin, LOW);
    digitalWrite(direction_reverse_pin, LOW);
  } else {  // Left wheel stop
    digitalWrite(direction_forward_pin, LOW);
    digitalWrite(direction_reverse_pin, LOW);
  }

  // Set the direction of the motors
  #if 0
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  } else {  // Left wheel stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  #endif


  if ((abs(pwm_req) - pwm_out) > 16)
  {
    pwm_inc = 8;
  }
  else if ((pwm_out - abs(pwm_req)) > 16)
  {
    pwm_inc = 8;
  }
  else
  {
    pwm_inc = 1;
  }

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwm_req) > pwm_out) {
    pwm_out += pwm_inc;
  } else if (abs(pwm_req) < pwm_out) {
    pwm_out -= pwm_inc;
  } else {
    // reached calculated PWM, then start PID
    // not stop case, run PID
    if (pwm_req != 0) {
      trackError = (wheel_vel - req_vel) * 10.0;
      if (trackPID->Compute())  //true if PID has triggered
        pwm_out += trackAdjustValue;
    }
  }

  // Conditional operator to limit PWM output at the maximum
  pwm_out = (pwm_out > PWM_MAX) ? PWM_MAX : pwm_out;


  // PWM output cannot be less than 0
  pwm_out = (pwm_out < 0) ? 0 : pwm_out;

  if (pwm_out > 0) {
    // Set the PWM value on the pins
    ledcWrite(pwm_channel, pwm_out);
  } else {
    // Set the PWM value on the pins
    ledcWrite(pwm_channel, -pwm_out);
  }

}

void MonicaMotor::restart_pid()
{
  //reset and start again PID controller
  trackPID->SetMode(MANUAL);
  trackAdjustValue = 0.0;
  trackError = 0.0;
  trackPID->SetMode(AUTOMATIC);
}
