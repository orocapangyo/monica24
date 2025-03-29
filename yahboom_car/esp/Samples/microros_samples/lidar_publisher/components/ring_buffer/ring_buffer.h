#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

// 环形队列结构体
typedef struct
{  
    uint8_t *buffer; // 缓冲区
    uint16_t head; // 队首指针
    uint16_t tail; // 队尾指针
    uint16_t size; // 环形队列的尺寸
} Ring_Buffer_t;

// 创建队列
void RingBuffer_Init(Ring_Buffer_t* ringBuf, uint16_t capacity);
// 队列重置
void RingBuffer_Reset(Ring_Buffer_t *ringBuf);
// 销毁队列
void RingBuffer_Destory(Ring_Buffer_t *ringBuf);
// 获取队列的容量
uint16_t RingBuffer_Get_Capacity(Ring_Buffer_t *ringBuf);
// 获取环形队列已经存好的字节数
uint16_t RingBuffer_Get_Used_Count(Ring_Buffer_t *ringBuf);
// 获取队列的剩余的空闲字节
uint16_t RingBuffer_Get_Free_Count(Ring_Buffer_t *ringBuf);
// 队列是否为空
uint8_t RingBuffer_IsEmpty(Ring_Buffer_t *ringBuf);
// 队列是否已满
uint8_t RingBuffer_IsFull(Ring_Buffer_t *ringBuf);
// 根据索引号读取第i个元素
uint8_t RingBuffer_GetValue_ByIndex(Ring_Buffer_t *ringBuf, uint16_t index);
// 清除队列数据（读出环形缓冲区中所有已存数据）
void RingBuffer_Clean_Queue(Ring_Buffer_t *ringBuf);
// 写入队尾元素（压栈）
void RingBuffer_Push(Ring_Buffer_t *ringBuf, uint8_t value);
// 弹出队首元素（出栈）
uint8_t RingBuffer_Pop(Ring_Buffer_t *ringBuf);




// 读取单个字节
uint8_t RingBuffer_ReadByte(Ring_Buffer_t *ringBuf);
// 读取字节数组
void RingBuffer_ReadByteArray(Ring_Buffer_t *ringBuf, uint8_t* dest, uint16_t size);
// 读取有符号Short整数(两个字节)
int16_t RingBuffer_ReadShort(Ring_Buffer_t *ringBuf);
// 读取无符号Short整数(两个字节)
uint16_t RingBuffer_ReadUShort(Ring_Buffer_t *ringBuf);
// 读取有符号Long类型的整数(四个字节)
int32_t RingBuffer_ReadLong(Ring_Buffer_t *ringBuf);
// 读取无符号Long类型的整数(四个字节)
uint32_t RingBuffer_ReadULong(Ring_Buffer_t *ringBuf);
// 读取浮点数(四个字节)
float RingBuffer_ReadFloat(Ring_Buffer_t *ringBuf);


// 写入单个字节
void RingBuffer_WriteByte(Ring_Buffer_t *ringBuf, uint8_t value);
// 写入字节数组
void RingBuffer_WriteByteArray(Ring_Buffer_t *ringBuf, uint8_t* src, uint16_t size);
// 写入有符号Short整数(两个字节)
void RingBuffer_WriteShort(Ring_Buffer_t *ringBuf, int16_t value);
// 写入无符号Short整数(两个字节)
void RingBuffer_WriteUShort(Ring_Buffer_t *ringBuf, uint16_t value);
// 写入有符号Long类型的整数(四个字节)
void RingBuffer_WriteLong(Ring_Buffer_t *ringBuf, int32_t value);
// 写入无符号Long类型的整数(四个字节)
void RingBuffer_WriteULong(Ring_Buffer_t *ringBuf, uint32_t value);
// 写入浮点数(四个字节)
void RingBuffer_WriteFloat(Ring_Buffer_t *ringBuf, float value);


#ifdef __cplusplus
}
#endif
