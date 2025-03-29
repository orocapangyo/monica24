#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"

#define KEY_GPIO_BOOT0        0
#define KEY_GPIO_USER1        42


#define KEY_STATE_PRESS       1
#define KEY_STATE_RELEASE     0


void Key_Init(void);
uint8_t Key0_Read_State(void);
uint8_t Key1_Read_State(void);

#ifdef __cplusplus
}
#endif
