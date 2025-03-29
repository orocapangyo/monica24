#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 队列重置
void RingBuffer_Reset(Ring_Buffer_t *ringBuf)
{
    // 队首指针跟队尾指针都指向buffer的首个地址
    ringBuf->head = 0;
    ringBuf->tail = 0;
    // 数值初始化置为0
    for (uint16_t i = 0; i < ringBuf->size; i++)
    {
        ringBuf->buffer[i] = 0;
    }
}

// 创建队列
void RingBuffer_Init(Ring_Buffer_t *ringBuf, uint16_t capacity)
{
    // 尺寸size需要比容量capacity大一
    ringBuf->size = capacity + 1;

    // Buffer内存申请
    ringBuf->buffer = (uint8_t *)malloc(ringBuf->size);
    
    // 重置队列数据
    RingBuffer_Reset(ringBuf);
    // printf("[RingBuffer_Init] ring buffer init pbuffer=%p\n", ringBuf->buffer);
}

// 销毁队列
void RingBuffer_Destory(Ring_Buffer_t *ringBuf)
{
	free(ringBuf->buffer);
	free(ringBuf);
}

// 获得环形队列的容量
uint16_t RingBuffer_Get_Capacity(Ring_Buffer_t *ringBuf)
{
    return ringBuf->size - 1;
}

// 获取环形队列已经存好的字节数
uint16_t RingBuffer_Get_Used_Count(Ring_Buffer_t *ringBuf)
{
    if (ringBuf->head > ringBuf->tail)
    {
        return RingBuffer_Get_Capacity(ringBuf) - (ringBuf->head - ringBuf->tail - 1);
    }
    else
    {
        return ringBuf->tail - ringBuf->head;
    }
}

// 获取队列的剩余容量
uint16_t RingBuffer_Get_Free_Count(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Capacity(ringBuf) - RingBuffer_Get_Used_Count(ringBuf);
}

// 判断环形队列是否为空
uint8_t RingBuffer_IsEmpty(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Free_Count(ringBuf) == RingBuffer_Get_Capacity(ringBuf);
}

// 判断环形队列是否已满
uint8_t RingBuffer_IsFull(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Free_Count(ringBuf) == 0;
}

// 根据索引号读取第i个元素
uint8_t RingBuffer_GetValue_ByIndex(Ring_Buffer_t *ringBuf, uint16_t index)
{
    if (index >= RingBuffer_Get_Used_Count(ringBuf))
    {
        // 索引号超出了范围
        return 0;
    }
    uint16_t rbIdx = (ringBuf->head + index + 1) % ringBuf->size;
    return ringBuf->buffer[rbIdx];
}

// 清除队列数据（读出环形缓冲区中所有已存数据）
void RingBuffer_Clean_Queue(Ring_Buffer_t *ringBuf)
{
    while (RingBuffer_Get_Used_Count(ringBuf))
    {
        RingBuffer_Pop(ringBuf);
    }
}

// 弹出队首元素
uint8_t RingBuffer_Pop(Ring_Buffer_t *ringBuf)
{
    uint8_t temp = 0;
    if (!RingBuffer_IsEmpty(ringBuf))
    {
        ringBuf->head = (ringBuf->head + 1) % ringBuf->size;
        temp = ringBuf->buffer[ringBuf->head];
        // 弹出首元素后清零
        ringBuf->buffer[ringBuf->head] = 0;
    }
    return temp;
}

// 写入队尾元素
void RingBuffer_Push(Ring_Buffer_t *ringBuf, uint8_t value)
{
    if (RingBuffer_IsFull(ringBuf))
    {
        // 队列已经写满了, 只能先弹出队首的元素
        RingBuffer_Pop(ringBuf);
    }
    ringBuf->tail = (ringBuf->tail + 1) % ringBuf->size;
    ringBuf->buffer[ringBuf->tail] = value;

    // if (!RingBuffer_IsFull(ringBuf))
    // {
    //     ringBuf->tail = (ringBuf->tail + 1) % ringBuf->size;
    //     ringBuf->buffer[ringBuf->tail] = value;
    // }
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

// 读取单个字节
uint8_t RingBuffer_ReadByte(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Pop(ringBuf);
}

// 读取字节数组
void RingBuffer_ReadByteArray(Ring_Buffer_t *ringBuf, uint8_t *dest, uint16_t size)
{
    for (uint16_t idx = 0; idx < size; idx++)
    {
        dest[idx] = RingBuffer_Pop(ringBuf);
    }
}

// 写入单个字节
void RingBuffer_WriteByte(Ring_Buffer_t *ringBuf, uint8_t value)
{
    RingBuffer_Push(ringBuf, value);
}

// 写入字节数组
void RingBuffer_WriteByteArray(Ring_Buffer_t *ringBuf, uint8_t *src, uint16_t size)
{
    for (uint16_t idx = 0; idx < size; idx++)
    {
        RingBuffer_Push(ringBuf, src[idx]);
    }
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////


// 读取有符号Short整数(两个字节)
int16_t RingBuffer_ReadShort(Ring_Buffer_t *ringBuf)
{
    int16_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 2);
    return value;
}

// 读取无符号Short整数(两个字节)
uint16_t RingBuffer_ReadUShort(Ring_Buffer_t *ringBuf)
{
    uint16_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 2);
    return value;
}

// 读取有符号Long类型的整数(四个字节)
int32_t RingBuffer_ReadLong(Ring_Buffer_t *ringBuf)
{
    int32_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}

// 读取无符号Long类型的整数(四个字节)
uint32_t RingBuffer_ReadULong(Ring_Buffer_t *ringBuf)
{
    uint32_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}

// 读取浮点数(四个字节)
float RingBuffer_ReadFloat(Ring_Buffer_t *ringBuf)
{
    float value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}


// 写入有符号Short整数(两个字节)
void RingBuffer_WriteShort(Ring_Buffer_t *ringBuf, int16_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// 写入无符号Short整数(两个字节)
void RingBuffer_WriteUShort(Ring_Buffer_t *ringBuf, uint16_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// 写入有符号Long类型的整数(四个字节)
void RingBuffer_WriteLong(Ring_Buffer_t *ringBuf, int32_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// 写入无符号Long类型的整数(四个字节)
void RingBuffer_WriteULong(Ring_Buffer_t *ringBuf, uint32_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// 写入浮点数(四个字节)
void RingBuffer_WriteFloat(Ring_Buffer_t *ringBuf, float value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}
