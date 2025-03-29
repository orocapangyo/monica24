#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define BEEP_GPIO           46

// 蜂鸣器有效电平，0=低电平响，1=高电平响
// Buzzer active level, 0= Active low, 1= Active high
#define BEEP_ACTIVE_LEVEL  1

#define BEEP_STATE_OFF       0
#define BEEP_STATE_ON_ALWAYS 1
#define BEEP_STATE_ON_DELAY  2


#define BEEP_ON()       Beep_On()
#define BEEP_OFF()      Beep_Off()


void Beep_Init(void);
void Beep_On(void);
void Beep_Off(void);
void Beep_On_Time(uint16_t time);

void Beep_Handle(void);

#ifdef __cplusplus
}
#endif
