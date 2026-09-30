/* breeze/include/drivers/remote/rc_common.h */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_REMOTE_RC_COMMON_H_
#define DRIVERS_REMOTE_RC_COMMON_H_

#include <drivers/remote/remote.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	把各按键的长按阈值写入 rc_sensor_info_t
 *
 * 只在设备初始化时调用一次；阈值属于配置，不随帧更新。
 */
void rc_keyboard_cnt_max_set(rc_sensor_t* sensor);

/**
 * @brief	遍历所有按键，逐个跑状态机
 */
void rc_keyboard_update(rc_sensor_info_t* info);

/**
 * @brief	数据合法性检查
 *
 * 通道越界（|ch| > 660）时置 DEV_DATA_ERR 并清零数据；
 * 同时由拨轮峰值判定 4 路 step 的翻转。所有结果就地写回 sensor->info。
 */
void rc_sensor_check(rc_sensor_t* sensor);

/**
 * @brief	鼠标速度均值滤波
 *
 * 每帧把 mouse_vx/mouse_vy 推入滑窗，用增量方式维护 mouse_x/mouse_y。
 */
void rc_interrupt_update(rc_sensor_t* sensor);

/**
 * @brief	心跳：累加掉线计数并维护 is_online
 */
void rc_sensor_heart_beat(rc_sensor_t* sensor);

/**
 * @brief	清空一帧数据
 *
 * 只清信号量，不动 tw_step_value / offline_max_cnt 这类配置字段。
 */
void rc_reset_data(rc_sensor_t* sensor);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_REMOTE_RC_COMMON_H_ */
