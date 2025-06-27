#ifndef _MONICA_MOTOR_H_
#define _MONICA_MOTOR_H_

#include <PID_v1.h>

#ifndef MOTOR_TYPE
    #define MOTOR_60RPM 1
    #define MOTOR_178RPM 2
    #define MOTOR_TYPE MOTOR_60RPM
#endif


#if MOTOR_TYPE == MOTOR_60RPM
#define K_P 1153.0
#define K_b 10
#define PWM_MIN 44.0   // about 0.03 m/s
#define PWM_MAX 194.0  // about 0.16 m/s
#define TICKS_PER_REVOLUTION (1860.0)
#define K_Lbias (0.0)
#else
#define K_P 431.0
#define K_b 5
#define PWM_MIN 22.0   // about 0.04m/s
#define PWM_MAX 82.0   // about 0.18 m/s
#define TICKS_PER_REVOLUTION (620.0)
#define K_Lbias (-3)
#endif


#define WHEEL_BASE (0.160)



class MonicaMotor
{
public:
    MonicaMotor(int _enc_in_pin_a, 
                int _enc_in_pin_b,
                int _forward_pin, 
                int _reverse_pin,
                int _standby_pin,
                int _pwm_pin,
                int _pwm_channel);
    ~MonicaMotor();

public:
    void initialize();
    void wheel_tick();
    void calc_vel();
    void set_pwm_values();
    void restart_pid();


public:
    int get_enc_in_pin_a() { return enc_in_pin_a; }
    int get_wheel_tick_count() { return wheel_tick_count; }
    void set_req_vel(float _req_vel) { req_vel = _req_vel; }
    void set_pwm_req(int _pwm_req) { pwm_req = _pwm_req; }

private:
    int enc_in_pin_a;
    int enc_in_pin_b;
    int direction_forward_pin;
    int direction_reverse_pin;
    int standby_pin;
    int pwm_pin;
    int pwm_channel;


    bool direction;
    volatile int wheel_tick_count;
    int prev_tick_count;
    float prev_time;
    
    float wheel_vel;
    float req_vel;

    int pwm_req;
    int pwm_out;


    PID* trackPID;
    double trackAdjustValue;
    double trackSetpoint;
    double trackError;


};

#endif /*_MONICA_MOTOR_H_*/
