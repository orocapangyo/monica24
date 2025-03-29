#include "ms200.h"
#include "math.h"
#include "string.h"

#include "esp_log.h"

#include "uart1.h"

#define ENABLE_DEBUG_MS200          0

#if ENABLE_DEBUG_MS200
#define YB_DEBUG     ESP_LOGI
#else
#define YB_DEBUG     ESP_LOGV
#endif



static const char *TAG = "MS200";


// CRC8数据表，多项式(0x4d):x^6 + x^3 + x^2 + 1
// CRC8 data table, polynomial(0x4d):x^6 + x^3 + x^2 + 1
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t new_package = 0;
char ms200_sn[50] = {0};
char ms200_version[100] = {0};

// 接收协议数据的缓存
// Cache for receiving protocol data
uint8_t rx_protocol_buf[MS200_BUF_MAX] = {0};
ms200_package_t ms200_pkg = {0};
ms200_data_t ms200_data = {0};



// 返回计算CRC8的值
// Returns the value of the calculated CRC8
static uint8_t Ms200_Calculate_CRC8(uint8_t* protocol_buf, uint8_t crc_len)
{
    int i;
    uint8_t crc = 0x00;
    for (i = 0; i < crc_len; i++)
    {
        crc = CRC_TABLE[(crc ^ protocol_buf[i]) & 0xFF];
    }
    return crc;
}


// 解析雷达系统报文, 输入协议缓存buf，输出雷达SN码信息或者版本信息，ASCII字符数组。
static void Ms200_Parse_Report(uint8_t* protocol_buf)
{
    int i;
    uint8_t data_len = protocol_buf[3];
    uint8_t crc8 = protocol_buf[data_len+4];
    if(protocol_buf[data_len+5] != MS200_TAIL_1 || protocol_buf[data_len+6] != MS200_TAIL_2) return;
    if (crc8 != Ms200_Calculate_CRC8(protocol_buf, data_len+4))
    {
        YB_DEBUG(TAG, "CRC8 Check Error");
        return;
    }
    
    // 判断SN码标志位，并保存SN码信息
    // Determine the SN flag bit and save the SN information
    if(protocol_buf[2] == MS200_FLAG_SN)
    {
        for (i = 0; i < data_len; i++)
        {
            ms200_sn[i] = protocol_buf[4 + i];
        }
        ESP_LOGI(TAG, "SN:%s", ms200_sn);
    }
    // 判断版本标志位，并保存版本信息
    // Determine the version flag bit and save the version information
    else if (protocol_buf[2] == MS200_FLAG_VERSION)
    {
        for (i = 1; i < data_len; i++)
        {
            ms200_version[i-1] = protocol_buf[4 + i];
        }
        ESP_LOGI(TAG, "%s", ms200_version);
    }
}

// 解析激光雷达点云数据包
// Analyze Lidar points data packets
static int Ms200_Parse_Package(uint8_t* protocol_buf, ms200_package_t* out_pkg)
{
    uint8_t buf_len = (protocol_buf[1] & 0x1F) * 3 + 11;
    uint8_t check_num = protocol_buf[buf_len-1];
    uint8_t crc8 = Ms200_Calculate_CRC8(protocol_buf, buf_len-1);
    if (crc8 != check_num)
    {
        YB_DEBUG(TAG, "CRC Error:%d, %d", crc8, check_num);
        return ESP_FAIL;
    }

    out_pkg->header = protocol_buf[0];
    out_pkg->count = protocol_buf[1] & 0x1F;
    out_pkg->speed = (protocol_buf[3] << 8) | protocol_buf[2];
    out_pkg->start_angle = (protocol_buf[5] << 8) | protocol_buf[4];
    out_pkg->end_angle = (protocol_buf[buf_len-4] << 8) | protocol_buf[buf_len-5];
    out_pkg->time_stamp = (protocol_buf[buf_len-2] << 8) | protocol_buf[buf_len-3];
    out_pkg->crc8 = protocol_buf[buf_len-1];

    for (int i = 0; i < MS200_POINT_PER_PACK; i++)
    {
        out_pkg->points[i].distance = (protocol_buf[3*i+7] << 8) | protocol_buf[3*i+6];
        out_pkg->points[i].intensity = protocol_buf[3*i+8];
    }
    return ESP_OK;
}

// 提取协议包数据，更新雷达数据包
// Extract protocol packet data and update radar packet
static void Ms200_Update_Data(ms200_package_t* pkg, ms200_data_t* out_data)
{
    uint16_t step_angle = 0;
    uint16_t angle = 0;
    if (pkg->end_angle > pkg->start_angle)
    {
        // 正常情况 normal codition
        step_angle = (pkg->end_angle - pkg->start_angle) / (pkg->count - 1);
    }
    else
    {
        // 特殊情况（0度）：结束角度小于开始角度
        // Special case: The end Angle is smaller than the start Angle
        step_angle = (36000 + pkg->end_angle - pkg->start_angle) / (pkg->count - 1);
    }
    for (int i = 0; i < pkg->count; i++)
    {
        angle = ((pkg->start_angle + i * step_angle) / 100) % MS200_POINT_MAX;
        out_data->points[angle].distance = pkg->points[i].distance;
        out_data->points[angle].intensity = pkg->points[i].intensity;
    }
}



// 激光雷达接收数据并处理
// The lidar receives and processes data
void Ms200_Data_Receive(uint8_t rx_data)
{
    static uint8_t rx_flag = 0;
    static uint8_t rx_buf_len = 0;
    static uint8_t rx_buf_index = 0;
    
    switch (rx_flag)
    {
    case 0:
    {
        if (rx_data == MS200_HEAD_1)
        {
            rx_flag = 1;
            rx_protocol_buf[0] = MS200_HEAD_1;
        }
        else if (rx_data == MS200_DATA_START)
        {
            // 开始接收点云数据
            // Start receiving points data
            rx_flag = 5;
            rx_protocol_buf[0] = MS200_DATA_START;
        }
        break;
    }
    case 1:
    {
        if (rx_data == MS200_HEAD_2)
        {
            rx_flag = 2;
            rx_protocol_buf[1] = MS200_HEAD_2;
        }
        else
        {
            rx_flag = 0;
        }
        break;
    }
    case 2: // 系统报文标志 System message flag
    {
        rx_protocol_buf[2] = rx_data;
        rx_flag = 3;
        break;
    }
    case 3: // 数据长度 data length
    {
        rx_protocol_buf[3] = rx_data;
        rx_flag = 4;
        rx_buf_len = rx_data+3;
        rx_buf_index = 0;
        break;
    }
    // 数据,校验位,桢尾
    // Data, check, end of frame
    case 4:
    {
        rx_protocol_buf[rx_flag + rx_buf_index] = rx_data;
        rx_buf_index++;
        // 雷达数据接收完成（一帧）
        // Lidar data reception complete(One frame)
        if (rx_buf_index >= rx_buf_len)
        {
            rx_flag = 0;
            rx_buf_len = 0;
            rx_buf_index = 0;

            Ms200_Parse_Report(rx_protocol_buf);
            memset(rx_protocol_buf, 0, sizeof(rx_protocol_buf));
        }
        // 超过最大数据，接收失败。 
        // If the rx_buf_index exceeds the maximum, receiving failed.
        if (rx_flag + rx_buf_index >= MS200_BUF_MAX)
        {
            rx_flag = 0;
            rx_buf_len = 0;
            rx_buf_index = 0;
            memset(rx_protocol_buf, 0, sizeof(rx_protocol_buf));
        }
        break;
    }
    case 5: // 点云数据点数N Point data N
    {
        rx_protocol_buf[1] = rx_data;
        rx_flag = 6;
        rx_buf_index = 2;
        rx_buf_len = (rx_protocol_buf[1]&0x1F)*3+11;
        break;
    }
    // 转速，起始角度，点云信息，结束角度，时间戳，校验位
    // Speed, start Angle, point information, end Angle, time stamp, check
    case 6:
    {
        rx_protocol_buf[rx_buf_index] = rx_data;
        rx_buf_index++;
        // 激光雷达数据接收完成（一帧）
        // Lidar data reception complete(One frame)
        if (rx_buf_index >= rx_buf_len)
        {
            rx_flag = 0;
            rx_buf_len = 0;
            rx_buf_index = 0;

            if (Ms200_Parse_Package(rx_protocol_buf, &ms200_pkg) == ESP_OK)
            {
                Ms200_Update_Data(&ms200_pkg, &ms200_data);
                new_package = 1;
            }
            memset(rx_protocol_buf, 0, sizeof(rx_protocol_buf));
        }
        // 超过最大数据，接收失败。
        // If the rx_buf_index exceeds the maximum, receiving failed.
        if (rx_buf_index >= MS200_BUF_MAX)
        {
            rx_flag = 0;
            rx_buf_len = 0;
            rx_buf_index = 0;
            memset(rx_protocol_buf, 0, sizeof(rx_protocol_buf));
        }
        break;
    }
    default:
        rx_flag = 0;
        rx_buf_len = 0;
        rx_buf_index = 0;
        break;
    }
}

// 激光雷达是否产生新数据
// Whether the lidar generates new data
uint8_t Ms200_New_Package(void)
{
    return new_package;
}

// 清除新数据状态
// Clear new data status
void Ms200_Clear_New_Package_State(void)
{
    new_package = 0;
}

// 获取MS200激光雷达的数据
// Get MS200 lidar data
void Ms200_Get_Data(ms200_data_t* out_data)
{
    *out_data = ms200_data;
}
