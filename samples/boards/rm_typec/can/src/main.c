/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_sample, LOG_LEVEL_INF);

/* 线程堆栈大小和优先级定义 */
#define STATE_POLL_THREAD_STACK_SIZE 512  // 状态轮询线程堆栈大小
#define STATE_POLL_THREAD_PRIORITY 2      // 状态轮询线程优先级

/* CAN消息ID定义 */
#define CAN1_MSG_ID CONFIG_CAN1_MSG_ID    // CAN1发送的消息ID
#define CAN2_MSG_ID CONFIG_CAN2_MSG_ID    // CAN2发送的消息ID

/* 时间间隔 */
#define SLEEP_TIME K_MSEC(CONFIG_SLEEP_TIME_MS)  // 发送间隔配置

/* 定义线程堆栈 */
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);  // 状态轮询线程堆栈

/* 获取CAN设备句柄 */
const struct device *const can1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));  // CAN1设备
const struct device *const can2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));  // CAN2设备

/* LED GPIO配置 */
struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});

/* 线程和工作队列相关结构体 */
struct k_thread poll_state_thread_data;   // 状态轮询线程数据
struct k_work state_change_work;          // 状态变化工作队列
enum can_state current_state;             // 当前CAN状态
struct can_bus_err_cnt current_err_cnt;   // 当前错误计数

#define CAN_SEND_QUEUE_SIZE 8

struct can_send_item {
	const struct device *dev;
	struct can_frame frame;
};

K_MSGQ_DEFINE(can_send_msgq, sizeof(struct can_send_item), CAN_SEND_QUEUE_SIZE, 4);

// 发送线程
#define CAN_SEND_THREAD_STACK_SIZE 512
#define CAN_SEND_THREAD_PRIORITY 2
K_THREAD_STACK_DEFINE(can_send_stack, CAN_SEND_THREAD_STACK_SIZE);
struct k_thread can_send_thread_data;

void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		LOG_ERR("TX callback error! Error code: %d, Sender: %s", error, sender);
	}
}

void can_send_thread(void *p1, void *p2, void *p3)
{
	struct can_send_item item;
	int ret;
	while (1) {
		k_msgq_get(&can_send_msgq, &item, K_FOREVER);
		do {
			ret = can_send(item.dev, &item.frame, K_FOREVER, tx_irq_callback, NULL);
			if (ret != 0) {
				LOG_WRN("CAN send failed, retrying... [%d]", ret);
				k_sleep(K_MSEC(10));
			}
		} while (ret != 0);
	}
}

/**
 * @brief CAN1接收回调函数
 * @param dev CAN设备指针
 * @param frame 接收到的CAN帧
 * @param user_data 用户数据
 */
void can1_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* 跳过RTR帧 */
	if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) && (frame->flags & CAN_FRAME_RTR) != 0U) {
		return;
	}

	LOG_INF("CAN1 received message: ID=0x%X, data[0]=0x%02X", frame->id, frame->data[0]);
	
	/* 如果是LED0控制消息，控制LED0 */
	if (frame->id == CAN2_MSG_ID && led0.port != NULL) {
		gpio_pin_set(led0.port, led0.pin, frame->data[0] == 1 ? 1 : 0);
		LOG_DBG("LED0 %s", frame->data[0] == 1 ? "ON" : "OFF");
	}
}

/**
 * @brief CAN2接收回调函数
 * @param dev CAN设备指针
 * @param frame 接收到的CAN帧
 * @param user_data 用户数据
 */
void can2_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* 跳过RTR帧 */
	if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) && (frame->flags & CAN_FRAME_RTR) != 0U) {
		return;
	}

	LOG_INF("CAN2 received message: ID=0x%X, data[0]=0x%02X", frame->id, frame->data[0]);
	
	/* 如果是LED1控制消息，控制LED1 */
	if (frame->id == CAN1_MSG_ID && led1.port != NULL) {
		gpio_pin_set(led1.port, led1.pin, frame->data[0] == 1 ? 1 : 0);
		LOG_DBG("LED1 %s", frame->data[0] == 1 ? "ON" : "OFF");
	}
}

/**
 * @brief CAN状态转换为字符串
 * @param state CAN状态枚举值
 * @return 状态字符串描述
 */
char *state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

/**
 * @brief CAN状态轮询线程
 * 定期检查CAN控制器的状态和错误计数
 */
void poll_state_thread(void *unused1, void *unused2, void *unused3)
{
	struct can_bus_err_cnt err_cnt = {0, 0};      // 当前错误计数
	struct can_bus_err_cnt err_cnt_prev = {0, 0}; // 上次错误计数
	enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;  // 上次状态
	enum can_state state;                         // 当前状态
	int err;

	while (1) {
		/* 获取CAN1的状态和错误计数 */
		err = can_get_state(can1_dev, &state, &err_cnt);
		if (err != 0) {
			LOG_ERR("Failed to get CAN controller state: %d", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		/* 检查状态或错误计数是否发生变化 */
		if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
		    err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt ||
		    state_prev != state) {

			/* 更新上次记录的值 */
			err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
			err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
			state_prev = state;
			
			/* 打印状态变化信息 */
			LOG_WRN("CAN state: %s, RX error count: %d, TX error count: %d",
			       state_to_str(state), err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
		} else {
			/* 没有变化时等待100ms */
			k_sleep(K_MSEC(100));
		}
	}
}

/**
 * @brief CAN状态变化工作队列处理函数
 * 在中断上下文中处理状态变化
 */
void state_change_work_handler(struct k_work *work)
{
	LOG_ERR("State change ISR - State: %s, RX error count: %d, TX error count: %d",
		state_to_str(current_state), current_err_cnt.rx_err_cnt, current_err_cnt.tx_err_cnt);
}

/**
 * @brief CAN状态变化回调函数
 * 当CAN状态发生变化时被调用
 */
void state_change_callback(const struct device *dev, enum can_state state,
			   struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct k_work *work = (struct k_work *)user_data;

	ARG_UNUSED(dev);

	current_state = state;
	current_err_cnt = err_cnt;
	k_work_submit(work);
}

/**
 * @brief 主函数
 * 初始化CAN控制器和GPIO，启动通信测试
 */
int main(void)
{
	/* 定义CAN1发送的帧结构 */
	struct can_frame can1_frame = {
		.flags = 0,                       // 标准帧
		.id = CAN1_MSG_ID,               // 消息ID为0x11
		.dlc = 1                         // 数据长度为1字节
	};
	
	/* 定义CAN2发送的帧结构 */
	struct can_frame can2_frame = {
		.flags = 0,                       // 标准帧
		.id = CAN2_MSG_ID,               // 消息ID为0x22
		.dlc = 1                         // 数据长度为1字节
	};
	
	k_tid_t get_state_tid;                // 线程ID
	int ret;                              // 返回值
	struct can_filter filter;             // CAN过滤器

	/* 检查CAN1设备是否就绪 */
	if (!device_is_ready(can1_dev)) {
		LOG_ERR("CAN1: Device %s not ready", can1_dev->name);
		return 0;
	}
	
	/* 检查CAN2设备是否就绪 */
	if (!device_is_ready(can2_dev)) {
		LOG_ERR("CAN2: Device %s not ready", can2_dev->name);
		return 0;
	}

	/* 启动CAN1控制器 */
	ret = can_start(can1_dev);
	if (ret != 0) {
		LOG_ERR("Failed to start CAN1 controller [%d]", ret);
		return 0;
	}
	
	/* 启动CAN2控制器 */
	ret = can_start(can2_dev);
	if (ret != 0) {
		LOG_ERR("Failed to start CAN2 controller [%d]", ret);
		return 0;
	}

	/* 配置CAN接收过滤器和回调函数 */
	LOG_INF("Configuring CAN RX filters and callbacks");

	/* 设置CAN1接收过滤器 - 接收所有消息 */
	filter.id = 0;
	filter.mask = 0;
	filter.flags = 0;

	ret = can_add_rx_filter(can1_dev, can1_rx_callback, NULL, &filter);
	if (ret < 0) {
		LOG_ERR("Failed to add CAN1 RX filter: %d", ret);
	} else {
		LOG_INF("CAN1 RX filter added successfully");
	}

	/* 设置CAN2接收过滤器 - 接收所有消息 */
	ret = can_add_rx_filter(can2_dev, can2_rx_callback, NULL, &filter);
	if (ret < 0) {
		LOG_ERR("Failed to add CAN2 RX filter: %d", ret);
	} else {
		LOG_INF("CAN2 RX filter added successfully");
	}

	/* 初始化LED0 GPIO */
	if (led0.port != NULL) {
		if (!gpio_is_ready_dt(&led0)) {
			LOG_ERR("LED0: Device %s not ready", led0.port->name);
		} else {
			ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_LOW);
			if (ret < 0) {
				LOG_ERR("Failed to configure LED0 pin as output [%d]", ret);
				led0.port = NULL;
			} else {
				LOG_INF("LED0 initialized successfully");
			}
		}
	}

	/* 初始化LED1 GPIO */
	if (led1.port != NULL) {
		if (!gpio_is_ready_dt(&led1)) {
			LOG_ERR("LED1: Device %s not ready", led1.port->name);
		} else {
			ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_LOW);
			if (ret < 0) {
				LOG_ERR("Failed to configure LED1 pin as output [%d]", ret);
				led1.port = NULL;
			} else {
				LOG_INF("LED1 initialized successfully");
			}
		}
	}

	/* 初始化工作队列 */
	k_work_init(&state_change_work, state_change_work_handler);

	/* 创建状态轮询线程 */
	get_state_tid = k_thread_create(&poll_state_thread_data,
					poll_state_stack,
					K_THREAD_STACK_SIZEOF(poll_state_stack),
					poll_state_thread, NULL, NULL, NULL,
					STATE_POLL_THREAD_PRIORITY, 0,
					K_NO_WAIT);
	if (!get_state_tid) {
		LOG_ERR("Failed to create state polling thread");
	}

	/* 设置CAN1的状态变化回调函数 */
	can_set_state_change_callback(can1_dev, state_change_callback, &state_change_work);

	LOG_INF("Initialization complete. Starting CAN communication test...");

	// 启动发送线程
	k_tid_t send_tid = k_thread_create(&can_send_thread_data,
		can_send_stack,
		K_THREAD_STACK_SIZEOF(can_send_stack),
		can_send_thread, NULL, NULL, NULL,
		CAN_SEND_THREAD_PRIORITY, 0,
		K_NO_WAIT);
	if (!send_tid) {
		LOG_ERR("Failed to create CAN send thread");
	}

	while (1) {
		/* CAN1发送LED0关闭消息 */
		can1_frame.data[0] = 0;
		struct can_send_item item1 = { .dev = can1_dev, .frame = can1_frame };
		if (k_msgq_put(&can_send_msgq, &item1, K_NO_WAIT) != 0) {
			LOG_WRN("CAN1 send queue full, drop LED0 ON frame");
		}

		/* CAN2发送LED1开启消息 */
		can2_frame.data[0] = 1;
		struct can_send_item item2 = { .dev = can2_dev, .frame = can2_frame };
		if (k_msgq_put(&can_send_msgq, &item2, K_NO_WAIT) != 0) {
			LOG_WRN("CAN2 send queue full, drop LED1 ON frame");
		}

#if !IS_ENABLED(CONFIG_CAN_TEST_OVERLOAD)
		k_sleep(SLEEP_TIME);
#endif

		/* CAN1发送LED0开启消息 */
		can1_frame.data[0] = 1;
		item1.frame = can1_frame;
		if (k_msgq_put(&can_send_msgq, &item1, K_NO_WAIT) != 0) {
			LOG_WRN("CAN1 send queue full, drop LED0 OFF frame");
		}

		/* CAN2发送LED1关闭消息 */
		can2_frame.data[0] = 0;
		item2.frame = can2_frame;
		if (k_msgq_put(&can_send_msgq, &item2, K_NO_WAIT) != 0) {
			LOG_WRN("CAN2 send queue full, drop LED1 OFF frame");
		}

#if !IS_ENABLED(CONFIG_CAN_TEST_OVERLOAD)
		k_sleep(SLEEP_TIME);
#else
		k_sleep(K_NSEC(10));  
#endif
	}
}