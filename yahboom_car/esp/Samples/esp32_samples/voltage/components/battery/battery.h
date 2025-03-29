#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define BATTERY_GPIO           3


void Battery_Init(void);
float Battery_Get_Voltage(void);

#ifdef __cplusplus
}
#endif
