#ifndef _PIN_MAP_H_
#define _PIN_MAP_H_

#define MONICA_PRODUCT_ID   0x24
#define MOTOR_60RPM 1
#define MOTOR_178RPM 2
#define MOTOR_TYPE MOTOR_60RPM

// Encoder output to Arduino Interrupt pin. Tracks the tick count
#if MOTOR_TYPE == MOTOR_60RPM
#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34
#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35
#else
#define ENC_IN_LEFT_A 39
#define ENC_IN_RIGHT_A 35
#define ENC_IN_LEFT_B 36
#define ENC_IN_RIGHT_B 34
#endif
// Motor A, B control
#define AIN1 26
#define AIN2 25
#define BIN1 27
#define BIN2 14

// Motor speed control via PWM
#define ENA 32
#define ENB 33
#define ENA_CH 0
#define ENB_CH 1

#define STBY 4


//mpu6050 interrupt pin
#define MPU_INT 12


//buzzer control
#define   BUZZER_CH  2
#define   BUZZER     13

// LED control pins
#define   LED_L     19
#define   LED_R     18
#define   LED_F     5


#endif /*_PIN_MAP_H_*/
