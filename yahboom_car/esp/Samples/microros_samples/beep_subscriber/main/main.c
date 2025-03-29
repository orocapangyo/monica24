#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int16.h>

#include "beep.h"


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



rcl_subscription_t subscriber;
std_msgs__msg__UInt16 msg_beep;

static const char *TAG = "MAIN";


// 订阅者回调函数 
// Subscriber callback function
void beep_callback(const void * msgin)
{
	const std_msgs__msg__UInt16 * msg = (const std_msgs__msg__UInt16 *)msgin;
	printf("Beep: %d\n",  (int)  msg->data);
    // 根据接收到的数据控制蜂鸣器的状态
    // Control the state of the buzzer based on the received data
    Beep_On_Time(msg->data);
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
    
    // 创建节点
    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "beep_subscriber", ROS_NAMESPACE, &support));

    // 创建订阅者
    // Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"beep"));

    // 创建执行者，其中三个参数为执行者控制的数量，要大于或等于添加到执行者的订阅者和发布者数量。
    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 1;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // 添加订阅者到执行者
    // Add subscriber to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_beep, &beep_callback, ON_NEW_DATA));

    // 循环执行microROS任务
    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // 释放资源
    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}


void app_main(void)
{
    // 初始化蜂鸣器
    // Initialize the buzzer
    Beep_Init();

    // 初始化网络，连接WiFi信号
    // Initialize the network and connect the WiFi signal
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // 开启microROS任务
    // Start microROS tasks
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
