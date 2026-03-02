/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * CAN TX Manager Example Program
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <drivers/can_tx_manager.h>
#include <string.h>

LOG_MODULE_REGISTER(can_tx_example, 3);

/* 定义 CAN 总线ID */
#define CAN_TX_ID_TEST1 0x123
#define CAN_TX_ID_TEST2 0x456

static const struct device *can1_dev = NULL;
static const struct device *can2_dev = NULL;
static const struct device *can_mgr_dev = NULL;

/* 发送器注册ID*/
static int sender_event_id = -1;   /* 事件触发设备 */
static int sender_periodic_id = -1; /* 定频发送设备 */

/* 接收计数器 */
static volatile uint32_t rx_count = 0;

/**
 * @brief 事件触发发送器的回调函数 - 填充 CAN 帧数据
 * 该回调用于准备待发送的数据
 */
static int fill_event_buffer(struct can_frame *frame, void *user_data)
{
    if (frame == NULL) {
        LOG_ERR("Invalid frame pointer");
        return -EINVAL;
    }

    uint32_t *counter = (uint32_t *)user_data;

    /* 填充帧数据：第一个字节为计数器 */
    frame->data[0] = (uint8_t)((*counter) & 0xFF);
    frame->data[1] = (uint8_t)((*counter) >> 8) & 0xFF;
    frame->data[2] = 0x11;
    frame->data[3] = 0x22;
    frame->data[4] = 0x33;
    frame->data[5] = 0x44;
    frame->data[6] = 0x55;
    frame->data[7] = 0x66;

    LOG_INF("Event-triggered callback: filling buffer with counter=%u", *counter);
    return 0;
}

/**
 * @brief 定频发送器的回调函数 - 填充 CAN 帧数据
 */
static int fill_periodic_buffer(struct can_frame *frame, void *user_data)
{
    if (frame == NULL) {
        LOG_ERR("Invalid frame pointer");
        return -EINVAL;
    }

    uint32_t *periodic_counter = (uint32_t *)user_data;

    frame->data[0] = 0xAA;
    frame->data[1] = 0xBB;
    frame->data[2] = (uint8_t)((*periodic_counter) & 0xFF);
    frame->data[3] = (uint8_t)((*periodic_counter) >> 8) & 0xFF;
    frame->data[4] = 0xFF;
    frame->data[5] = 0xEE;
    frame->data[6] = 0xDD;
    frame->data[7] = 0xCC;

    /* (*periodic_counter)++; */
    return 0;
}

/**
 * @brief 发送完成回调
 */
static void send_callback(const struct device *dev, int error, void *user_data)
{
    if (error != 0) {
        LOG_ERR("Send failed with error: %d", error);
    } else {
        LOG_DBG("Frame sent successfully");
    }
}

/**
 * @brief CAN 接收线程 - 监听并打印接收到的数据
 */
/* RX callback for CAN2 */
static void can2_rx_callback(const struct device *dev, struct can_frame *frame,
                             void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (frame == NULL) {
        return;
    }

    rx_count++;
    LOG_INF("[RX #%u] ID: 0x%03X | DLC: %u | Flags: 0x%02X",
            rx_count, frame->id, frame->dlc, frame->flags);
    LOG_HEXDUMP_INF(frame->data, frame->dlc, "CAN Frame Data:");
}


/**
 * @brief 主线程 - 测试各项功能
 */
int main(void)
{
    can1_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(fdcan1));         // 只有喵板是fdcan，用节点名获取设备在应用层不是很好的选择，我只是偷懒了
    if (!can1_dev || !device_is_ready(can1_dev)) {
        LOG_ERR("fdcan1 device missing or not ready");
        return -ENODEV;
    }

    can2_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(fdcan2));
    if (!can2_dev) {
        LOG_INF("fdcan2 not defined, reusing fdcan1 for RX");    // 如果板子只有一路can则复用
    }
    else if (!device_is_ready(can2_dev)) {
        LOG_ERR("fdcan2 device not ready");
        return -ENODEV;
    }

    can_mgr_dev = device_get_binding("can_tx_mgr1");
    if (!device_is_ready(can_mgr_dev)) {
        LOG_ERR("CAN TX Manager device not ready");
        return -ENODEV;
    }

    /* 检查并启动 CAN1 */
    int start_ret = can_start(can1_dev);
    if (start_ret != 0) {
        LOG_ERR("Failed to start CAN1: %d", start_ret);
        return start_ret;
    }
    /* 检查并启动 CAN2（如果不同） */
    if (can2_dev != can1_dev) {
        start_ret = can_start(can2_dev);
        if (start_ret != 0) {
            LOG_ERR("Failed to start CAN2: %d", start_ret);
            return start_ret;
        }
    }

    // 添加接收过滤器，全开
    struct can_filter filter = {
        .id = 0x000,           /* 通配符：接收所有 ID */
        .mask = 0x000,         /* mask=0 表示接受任何 ID */
        .flags = 0,
    };
    int filt_ret = can_add_rx_filter(can2_dev, can2_rx_callback, NULL, &filter);
    if (filt_ret < 0) {
        LOG_ERR("Failed to add CAN2 RX filter: %d", filt_ret);
    }

    // 挂载设备到 TX manager，这个设备为手动发送
    static uint32_t event_counter = 0;
    sender_event_id = can_tx_manager_register(
        can_mgr_dev,
        CAN_TX_ID_TEST1,            /* tx_id */
        0x000,                      /* rx_id (unused in this example) */
        8,                            /* dlc */
        0,                          /* flags: standard frame */
        0,                      /* frequency: event-triggered */
        fill_event_buffer, /* callback */
        &event_counter                    /* user_data */
    );

    if (sender_event_id < 0) {
        LOG_ERR("Failed to register event-triggered sender: %d", sender_event_id);
        return sender_event_id;
    }
    LOG_INF("Event-triggered sender registered, ID: %d", sender_event_id);

    // 挂载设备到 TX manager，这个设备为定周期发送
    static uint32_t periodic_counter = 0;
    sender_periodic_id = can_tx_manager_register(
        can_mgr_dev,
        CAN_TX_ID_TEST2,                /* tx_id */
        0x000,                          /* rx_id */
        8,                                /* dlc */
        0,                              /* flags: standard frame */
        10,                         /* frequency: 10 Hz */
        fill_periodic_buffer,  /* callback */
        &periodic_counter                      /* user_data */
    );

    if (sender_periodic_id < 0) {
        LOG_ERR("Failed to register periodic sender: %d", sender_periodic_id);
        return sender_periodic_id;
    }
    LOG_INF("Periodic sender registered, ID: %d (10Hz)", sender_periodic_id);

    // 手动发送三条报文
    for (int i = 0; i < 3; i++) {
        event_counter = 100 + i;
        int ret = can_tx_manager_send(
            can_mgr_dev,
            K_MSEC(100),        /* timeout */
            send_callback,              /* callback */
            CAN_TX_ID_TEST1,     /* tx_id */
            NULL                        /* user_data for callback */
        );

        if (ret != 0) {
            LOG_ERR("Send failed: %d", ret);
        } else {
            LOG_INF("Event send #%d successful", i + 1);
        }

        k_sleep(K_MSEC(200));
    }

    // 注销设备1
    LOG_INF("\nTesting unregister...");

    int ret = can_tx_manager_unregister(
        can_mgr_dev,
        CAN_TX_ID_TEST1,
        0x000
    );

    if (ret != 0) {
        LOG_ERR("Unregister failed: %d", ret);
    } else {
        LOG_INF("Event-triggered sender unregistered successfully");
    }

    LOG_INF("\nTrying to send with unregistered sender (should fail)...");

    ret = can_tx_manager_send(
        can_mgr_dev,
        K_MSEC(100),
        send_callback,
        CAN_TX_ID_TEST1,
        NULL
    );

    if (ret != 0) {
        LOG_INF("Send correctly failed after unregister (expected): %d", ret);
    } else {
        LOG_WRN("Send unexpectedly succeeded, something is wrong");
    }

    // 注销设备2
    LOG_INF("\nTesting unregistering periodic sender...");

    ret = can_tx_manager_unregister(
        can_mgr_dev,
        CAN_TX_ID_TEST2,
        0x000
    );

    if (ret != 0) {
        LOG_ERR("Unregister periodic sender failed: %d", ret);
    } else {
        LOG_INF("Periodic sender unregistered successfully");
    }

    /* 最终统计 */
    k_sleep(K_MSEC(1000));
    LOG_INF("\n=== Test Complete ===");
    LOG_INF("Total frames received on CAN2: %u", rx_count);
    LOG_INF("Program finished");

    return 0;
}
