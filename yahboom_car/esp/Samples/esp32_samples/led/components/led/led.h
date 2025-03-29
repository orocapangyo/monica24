#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define LED_GPIO           45

// led电平，0=低电平点亮，1=高电平点亮
// led level, 0= low level on, 1= high level on
#define LED_ACTIVE_LEVEL  1


void Led_Init(void);
void Led_On(void);
void Led_Off(void);
void Led_State(uint8_t state);
void Led_Flash(uint16_t interval);

#ifdef __cplusplus
}
#endif
