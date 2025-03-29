#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define UART1_GPIO_TXD           (GPIO_NUM_17)
#define UART1_GPIO_RXD           (GPIO_NUM_18)

#define RX1_BUF_SIZE             (1024)



void Uart1_Init(void);
int Uart1_Send_Byte(uint8_t data);
int Uart1_Send_Data(uint8_t* data, uint16_t len);


uint16_t Uart1_Available(void);
uint8_t Uart1_Read(void);
void Uart1_Clean_Buffer(void);


#ifdef __cplusplus
}
#endif
