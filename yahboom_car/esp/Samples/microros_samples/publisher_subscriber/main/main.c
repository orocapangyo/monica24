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
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>


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


rcl_publisher_t publisher_1;
std_msgs__msg__Int32 msg_pub1;

rcl_publisher_t publisher_2;
std_msgs__msg__Int32 msg_pub2;

rcl_publisher_t publisher_3;
std_msgs__msg__Int32 msg_pub3;


rcl_timer_t timer_1;
rcl_timer_t timer_2;
rcl_timer_t timer_3;


rcl_subscription_t subscriber_1;
std_msgs__msg__Int32 msg_sub1;

rcl_subscription_t subscriber_2;
std_msgs__msg__Int32 msg_sub2;

rcl_subscription_t subscriber_3;
std_msgs__msg__Int32 msg_sub3;

// 定时器timer1回调函数
// Timer1 callback function
void timer1_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher_1, &msg_pub1, NULL));
        msg_pub1.data++;
    }
}

// 定时器timer2回调函数
// Timer2 callback function
void timer2_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher_2, &msg_pub2, NULL));
        msg_pub2.data++;
    }
}

// 定时器timer3回调函数
// Timer3 callback function
void timer3_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher_3, &msg_pub3, NULL));
        msg_pub3.data++;
    }
}

// 订阅者Subscriber_1回调函数 
// Subscriber_1 callback function
void subscription_1_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Sub1 Received: %d\n",  (int)  msg->data);
}

// 订阅者Subscriber_2回调函数 
// Subscriber_2 callback function
void subscription_2_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Sub2 Received: %d\n",  (int)  msg->data);
}

// 订阅者Subscriber_3回调函数 
// Subscriber_3 callback function
void subscription_3_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Sub3 Received: %d\n",  (int)  msg->data);
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
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    // 尝试连接代理，连接成功才进入下一步。
    // Try to connect to the agent. If the connection succeeds, go to the next step.
    int state_agent = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 创建ROS2节点
    // create ROS2 node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_pub_sub", ROS_NAMESPACE, &support));

    // 创建发布者1
    // create publisher_1
    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "publisher_1"));
    // 创建发布者2
    // create publisher_2
    RCCHECK(rclc_publisher_init_default(
        &publisher_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "publisher_2"));
    // 创建发布者3
    // create publisher_3
    RCCHECK(rclc_publisher_init_default(
        &publisher_3,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "publisher_3"));

    // 创建订阅者1
    // Create subscriber_1.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"subscriber_1"));
    // 创建订阅者2
    // Create subscriber_2.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"subscriber_2"));
    // 创建订阅者3
    // Create subscriber_3.
	RCCHECK(rclc_subscription_init_default(
		&subscriber_3,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"subscriber_3"));
    // 设置发布频率为10HZ
    const unsigned int timer1_timeout = 100;
    const unsigned int timer2_timeout = 100;
    const unsigned int timer3_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer_1,
        &support,
        RCL_MS_TO_NS(timer1_timeout),
        timer1_callback));

    RCCHECK(rclc_timer_init_default(
        &timer_2,
        &support,
        RCL_MS_TO_NS(timer2_timeout),
        timer2_callback));

    RCCHECK(rclc_timer_init_default(
        &timer_3,
        &support,
        RCL_MS_TO_NS(timer3_timeout),
        timer3_callback));

    // 创建执行者，其中三个参数为执行者控制的数量，要大于或等于添加到执行者的订阅者和发布者数量。
    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 6;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // 设置rclc执行者超时时间为1000毫秒
    // Setup the rclc executor timeout to 1000 ms
    unsigned int rcl_wait_timeout = 1000;
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
    // 添加发布者的定时器到执行者
    // Adds the publisher's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_3));
    // 添加订阅者到执行者
    // Add subscriber to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_1, &msg_sub1, &subscription_1_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_2, &msg_sub2, &subscription_2_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_3, &msg_sub3, &subscription_3_callback, ON_NEW_DATA));

    msg_pub1.data = 0;
    msg_pub2.data = 0;
    msg_pub3.data = 0;

    // 循环执行microROS任务
    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // 释放资源
    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_1, &node));
    RCCHECK(rcl_publisher_fini(&publisher_2, &node));
    RCCHECK(rcl_publisher_fini(&publisher_3, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_1, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_2, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_3, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
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
