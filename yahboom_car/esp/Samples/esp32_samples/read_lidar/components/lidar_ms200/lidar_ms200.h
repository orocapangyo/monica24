#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"


void Lidar_Ms200_Init(void);
uint16_t Lidar_Ms200_Get_Distance(uint16_t angle);

#ifdef __cplusplus
}
#endif
