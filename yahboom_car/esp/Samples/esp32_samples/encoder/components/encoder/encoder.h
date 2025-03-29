#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define ENCODER_GPIO_H1A            6
#define ENCODER_GPIO_H1B            7

#define ENCODER_GPIO_H2A            47
#define ENCODER_GPIO_H2B            48

#define ENCODER_GPIO_H3A            11
#define ENCODER_GPIO_H3B            12

#define ENCODER_GPIO_H4A            1
#define ENCODER_GPIO_H4B            2



#define ENCODER_PCNT_HIGH_LIMIT   1000
#define ENCODER_PCNT_LOW_LIMIT    -1000


typedef enum _encoder_id 
{
    ENCODER_ID_M1 = 1,
    ENCODER_ID_M2 = 2,
    ENCODER_ID_M3 = 3,
    ENCODER_ID_M4 = 4
} encoder_id;



void Encoder_Init(void);
int Encoder_Get_Count_M1(void);
int Encoder_Get_Count_M2(void);
int Encoder_Get_Count_M3(void);
int Encoder_Get_Count_M4(void);
int Encoder_Get_Count(uint8_t encoder_id);

#ifdef __cplusplus
}
#endif
