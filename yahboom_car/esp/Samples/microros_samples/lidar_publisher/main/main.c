#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <micro_ros_utilities/string_utilities.h>

#include "lidar_ms200.h"
#include "ms200.h"
#include "uart1.h"


#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }


#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT



static const char *TAG = "MAIN";


rcl_publisher_t publisher_lidar;
sensor_msgs__msg__LaserScan msg_lidar;
rcl_timer_t timer_lidar;

unsigned long long time_offset = 0;

// 初始化激光雷达的ROS话题信息
// Initializes the ROS topic information for lidar
void lidar_ros_init(void)
{
    int i;

    msg_lidar.angle_min = -180*M_PI/180.0;
    msg_lidar.angle_max = 180*M_PI/180.0;

    msg_lidar.angle_increment = 1*M_PI/180.0;
    msg_lidar.range_min = 0.12;
    msg_lidar.range_max = 8.0;

    msg_lidar.ranges.data = (float *)malloc(360 * sizeof(float));
    msg_lidar.ranges.size = 360;
    for (i = 0; i < msg_lidar.ranges.size; i++)
    {
        msg_lidar.ranges.data[i] = 0;
    }
    
    msg_lidar.intensities.data = (float *)malloc(360 * sizeof(float));
    msg_lidar.intensities.size = 360;
    for (i = 0; i < msg_lidar.intensities.size; i++)
    {
        msg_lidar.intensities.data[i] = 10.0;
    }

    char* content_frame_id = "laser_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "lidar frame len:%d", len_frame_id_max);
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        // ROS命名空间为空字符
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        // 拼接命名空间和frame id
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    msg_lidar.header.frame_id = micro_ros_string_utilities_set(msg_lidar.header.frame_id, frame_id);
    free(frame_id);
}

// 激光雷达更新数据任务
// lidar update data task
void lidar_update_data_task(void *arg)
{
    uint16_t distance_mm[MS200_POINT_MAX] = {0};
    uint8_t intensity[MS200_POINT_MAX] = {0};
    uint16_t index = 0;
    int i = 0;
    while (1)
    {
        index = 0;
        for (i = 0; i < MS200_POINT_MAX; i++)
        {
            distance_mm[i] = Lidar_Ms200_Get_Distance(i);
            intensity[i] = Lidar_Ms200_Get_Intensity(i);
        }
        for (i = 0; i < MS200_POINT_MAX; i++)
        {
            index = (MS200_POINT_MAX-i) % MS200_POINT_MAX;
            if (index >= 180)
            {
                index = (index - 180) % MS200_POINT_MAX;
            }
            else
            {
                index = (index + 180) % MS200_POINT_MAX;
            }
            msg_lidar.ranges.data[i] = (float)(distance_mm[index] / 1000.0);
            msg_lidar.intensities.data[i] = (float)(intensity[index]);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}


// 获取从开机到现在的秒数
// Gets the number of seconds since boot
unsigned long get_millisecond(void)
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

// 计算microROS代理与MCU的时间差
// Calculate the time difference between the microROS agent and the MCU
static void sync_time(void)
{
    unsigned long now = get_millisecond();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

// 获取时间戳
// Get timestamp
struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    // 同步时间 deviation of synchronous time
    unsigned long long now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}


// 定时器回调函数
// Timer callback function
void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_lidar.header.stamp.sec = time_stamp.tv_sec;
        msg_lidar.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_lidar, &msg_lidar, NULL));
    }
}

// micro_ros处理任务 
// micro ros processes tasks
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // 创建rcl初始化选项
    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // 修改ROS域ID
    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // 初始化rmw选项
    // Initialize the rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // 设置静态代理IP和端口
    // Setup static agent IP and port
    RCCHECK(rmw_uros_options_set_udp_address(ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    // 尝试连接代理，连接成功才进入下一步。
    // Try to connect to the agent. If the connection succeeds, go to the next step.
    int state_agent = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 创建ROS2节点
    // create ROS2 node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "lidar_publisher", ROS_NAMESPACE, &support));

    // 创建发布者
    // create publisher_lidar
    RCCHECK(rclc_publisher_init_default(
        &publisher_lidar,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"));

    // 创建定时器，设置发布频率为10HZ
    // create timer. Set the publish frequency to 11HZ
    const unsigned int timer_timeout = 90;
    RCCHECK(rclc_timer_init_default(
        &timer_lidar,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_lidar_callback));

    // 创建执行者，其中三个参数为执行者控制的数量，要大于或等于添加到执行者的订阅者和发布者数量。
    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 1;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // 添加发布者的定时器到执行者
    // Adds the publisher_lidar's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_lidar));

    sync_time();

    // 循环执行microROS任务
    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // 释放资源
    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_lidar, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    // 初始化串口1
    // Initialize serial port 1
    Uart1_Init();

    // 初始化激光雷达
    // Initialize the Lidar
    Lidar_Ms200_Init();

    // 初始化网络，连接WiFi信号
    // Initialize the network and connect the WiFi signal
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    lidar_ros_init();

    // 开启microROS任务
    // Start microROS tasks
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    // 开启激光雷达更新数据任务
    // Start lidar tasks
    xTaskCreatePinnedToCore(lidar_update_data_task,
                "lidar_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}
