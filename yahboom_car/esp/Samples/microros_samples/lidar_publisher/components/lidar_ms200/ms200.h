#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"

#define MS200_HEAD_1                0xAA
#define MS200_HEAD_2                0x55
#define MS200_TAIL_1                0x31
#define MS200_TAIL_2                0xF2
#define MS200_FLAG_SN               0x01
#define MS200_FLAG_VERSION          0x02

#define MS200_DATA_START            0x54


#define MS200_POINT_MAX  		    360  // 雷达点数，一圈360个点
#define MS200_BUF_MAX               100  // 雷达接收buf最大长度
#define MS200_POINT_PER_PACK        12   // 每包协议中的点数


// 雷达点云数据
typedef struct __attribute__((packed))
{
    uint16_t distance; // 距离(mm)
    uint8_t intensity; // 强度
} ms200_point_t;

// MS200雷达协议数据包 MS200 lidar protocol packet
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t count;
    uint16_t speed;
    uint16_t start_angle;
    ms200_point_t points[MS200_POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t time_stamp;
    uint8_t crc8;
} ms200_package_t;

// ms200雷达数据 ms200 lidar data
typedef struct __attribute__((packed))
{
    uint16_t size;
    ms200_point_t points[MS200_POINT_MAX];
} ms200_data_t;



void Ms200_Data_Receive(uint8_t rxtemp);
uint8_t Ms200_New_Package(void);
void Ms200_Clear_New_Package_State(void);
void Ms200_Get_Data(ms200_data_t* out_data);



#ifdef __cplusplus
}
#endif
