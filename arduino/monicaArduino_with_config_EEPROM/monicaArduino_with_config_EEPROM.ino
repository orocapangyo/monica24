#include <Arduino.h>

#include <stdio.h>
//#include <rcl/error_handling.h>
//#include <rmw_microros/rmw_microros.h>

//#include <std_msgs/msg/float32.h>

#include "PinMap.h"

#include "MonicaSong.h"
#include "MonicaDisplay.h"
#include "MonicaLed.h"

#include "MonicaMotor.h"
#include "MotorModule.h"

#include "MonicaMPU6050.h"
#include "MonicaMPUModule.h"

#include "MonicaRosComm.h"
#include "MonicaEEPROM.h"
#include "MonicaConfigComm.h"
#include "MonicaCommModule.h"

#define PRINT_VEL 0
#define PRINT_PIDERR 0
#define PRINT_AVGPWM 0
#define PRINT_INSPWM 0
#define PRINT_QUAT 0


#define RXD2 16
#define TXD2 17

#define DEBUG 0
#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial2.print(x)
#define DEBUG_PRINTY(x, y) Serial2.print(x, y)
#define DEBUG_PRINTLN(x) Serial2.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define ENABLE_MPU

MonicaSong song(BUZZER, BUZZER_CH);
MonicaDisplay display;
MonicaLed led(LED_L, LED_R, LED_F);

MonicaMotor motor_left(ENC_IN_LEFT_A, ENC_IN_LEFT_B, AIN1, AIN2, STBY, ENA, ENA_CH);
MonicaMotor motor_right(ENC_IN_RIGHT_A, ENC_IN_RIGHT_B, BIN1, BIN2, STBY, ENB, ENB_CH);
MotorModule motor_module(&motor_left, &motor_right);

MonicaMPU6050 mpu(MPU_INT);
MonicaMPUModule mpu_module(&mpu);


MonicaEEPROM eeprom_module;

MonicaRosComm ros_comm(&motor_left, &motor_right, &song, &display, &led, &mpu);
MonicaConfigComm config_comm(&eeprom_module);

MonicaCommModule comm_module(&config_comm, &ros_comm, &eeprom_module);


#define INTERVAL 50  //50ms, so 20Hz

long previousMillis = 0;
long currentMillis = 0;


void setup() {
  
#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif
  DEBUG_PRINTLN("Enc/Motor/MPU6050/Other Starts");
  
  // eeprom init first.
  eeprom_module.initialize();

  song.Initialize();
  display.Initialize();
  led.Initialize();

  motor_left.initialize();
  motor_right.initialize();
  motor_module.initialize();

#ifdef ENABLE_MPU
  mpu.initialize();
  mpu_module.initialize();
#endif

  comm_module.initialize();

  ros_comm.Initialize();
  //config_comm.initialize();

  DEBUG_PRINTLN("Done setup");
}

void loop() {
 
  ros_comm.check_agent_state();


  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    // Calculate the velocity of the right and left wheels
    motor_left.calc_vel();
    motor_right.calc_vel();

    mpu.check_value();

    if (config_comm.is_receive_config() == false)
    {
      ros_comm.ready_to_publish_message();
      ros_comm.publish_messages();
    }
    
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - ros_comm.get_last_cmd_vel_received_time() > 1) 
  {
    motor_left.set_pwm_req(0);
    motor_right.set_pwm_req(0);
  }

  motor_left.set_pwm_values();
  motor_right.set_pwm_values();

  song.play();
  display.show();

  if (ros_comm.is_used_wifi())
  {
    config_comm.receive();
  }
  
  config_comm.check_timeout();
}


