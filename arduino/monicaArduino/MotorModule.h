#ifndef _MOTOR_MODULE_H_
#define _MOTOR_MODULE_H_

#include "MonicaMotor.h"

class MotorModule
{
public:
    MotorModule(MonicaMotor* motor_left, 
                MonicaMotor* motor_right);
    ~MotorModule();

public:
    static void initialize();

public:
    void set_motor_left(MonicaMotor* motor_left) { motor_left_ = motor_left; }
    void set_motor_right(MonicaMotor* motor_right) { motor_right_ = motor_right; }


private:
    static void motor_left_encoder_interrupt();
    static void motor_right_encoder_interrupt();


private:
    static MonicaMotor* motor_left_;
    static MonicaMotor* motor_right_;
};


#endif /*_MOTOR_MODULE_H_*/
