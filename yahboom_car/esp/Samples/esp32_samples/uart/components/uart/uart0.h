#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define UART0_GPIO_TXD            (GPIO_NUM_43)
#define UART0_GPIO_RXD            (GPIO_NUM_44)

#define RX0_BUF_SIZE              (1024)



void Uart0_Init(void);
int Uart0_Send_Byte(uint8_t data);
int Uart0_Send_Data(uint8_t* data, uint16_t len);


uint16_t Uart0_Available(void);
uint8_t Uart0_Read(void);
void Uart0_Clean_Buffer(void);

#ifdef __cplusplus
}
#endif
