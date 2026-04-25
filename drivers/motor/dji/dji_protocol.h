/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef DJI_PROTOCOL_H
#define DJI_PROTOCOL_H

#include <drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>


#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif
#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif

#define LOG_LEVEL CONFIG_MOTOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_dji_can);


/* Fallbacks for static analysis (Zephyr builds define these via autoconf.h) */
#ifndef CONFIG_MOTOR_HEARTBEAT_OFFLINE_TIMEOUT_MS
#define CONFIG_MOTOR_HEARTBEAT_OFFLINE_TIMEOUT_MS 100
#endif
#ifndef CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS
#define CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS 10
#endif

/*
 * motor-id: DTS string -> const char*
 * control-mode: DTS enum -> DT_ENUM_IDX
 */
typedef struct motor_dji_cfg_t {
    uint16_t tx_id;
    uint16_t rx_id;
    int8_t motor_type;
    const char *motor_label;
    int8_t control_mode;
    uint16_t motor_encoder;
    uint8_t transmission_ratio;
    const struct device *can_dev;
#if defined(CONFIG_CAN_RX_MANAGER)
    const struct device *rx_mgr;            // 可选：接收管理器
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    const struct device *tx_mgr;            // 可选：发送管理器
#endif
} motor_dji_cfg_t;

typedef struct motor_dji_data_t
{
    smotor_data_t motor_data;
    uint16_t Tx_feq;                        // 发送频率，单位Hz，0表示仅手动发送
    struct k_spinlock lock;                 // 保护 motor_data 的自旋锁，防止接收更新和心跳检测冲突
    bool registered;
#if defined(CONFIG_CAN_RX_MANAGER)
    int rxmanager_slot_id;                  // CAN RX管理器 槽位ID
#endif

#if defined(CONFIG_MOTOR_HEARTBEAT_AUTOCHECK)
    const struct device *dev_self;          // 指向自身设备的指针，用于心跳自动检测
    struct k_work_delayable hb_work;        // 心跳自动检测的延时工作
#endif
} motor_dji_data_t;

static inline void motor_M3508_fillbuffer(motor_dji_data_t *data, const struct can_frame *frame)
{
    data->motor_data.rx_data.encoder = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
    data->motor_data.rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
    data->motor_data.rx_data.iq = (int16_t)((frame->data[4] << 8) | frame->data[5]);
    data->motor_data.rx_data.specific_data.m3508.temp = (int16_t)frame->data[6];
    data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
                                                     MOTOR_RX_VALID_SPEED |
                                                     MOTOR_RX_VALID_IQ |
                                                     MOTOR_DJI_6020);
}

static inline void motor_M2006_fillbuffer(motor_dji_data_t *data, const struct can_frame *frame)
{
    data->motor_data.rx_data.encoder = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
    data->motor_data.rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
    data->motor_data.rx_data.iq = (int16_t)((frame->data[4] << 8) | frame->data[5]);
    data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
                                                     MOTOR_RX_VALID_SPEED |
                                                     MOTOR_RX_VALID_IQ);
}

static inline void motor_M6020_fillbuffer(motor_dji_data_t *data, const struct can_frame *frame)
{
    data->motor_data.rx_data.encoder = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
    data->motor_data.rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
    data->motor_data.rx_data.iq = (int16_t)((frame->data[4] << 8) | frame->data[5]);
    data->motor_data.rx_data.specific_data.m6020.temp = (int16_t)frame->data[6];
    data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
                                                     MOTOR_RX_VALID_SPEED |
                                                     MOTOR_RX_VALID_IQ |
                                                     MOTOR_DJI_6020);
}


#endif