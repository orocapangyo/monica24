#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

// I2C设备地址
#define ICM42670P_I2C_ADDR        0x68
#define IMU_GPIO_INT              41


void Icm42670p_Init(void);
void Icm42670p_Get_Accel_RawData(int16_t accel[3]);
void Icm42670p_Get_Gyro_RawData(int16_t gyro[3]);
int Icm42670p_Start_OK(void);

void Icm42670p_Get_Accel_g(float accel_g[3]);
void Icm42670p_Get_Gyro_dps(float gyro_dps[3]);
void Icm42670p_Get_Accel_Gyro_FSR(uint16_t *accel_fsr_g, uint16_t *gyro_fsr_dps);

#ifdef __cplusplus
}
#endif
