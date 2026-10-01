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
 * 单位是 ms，与 rc_keyboard_update() 的 1ms 调用周期对应。
 */
void rc_keyboard_cnt_max_set(rc_sensor_t* sensor);

/**
 * @brief	遍历所有按键，逐个跑状态机
 *
 * 不在解析路径里自动调用：解析只负责把每帧的 value 填进 info，何时推进状态机
 * 由使用方决定。
 *
 * 调用周期要求 1ms 固定周期：每次调用给「已按住时长」+1，所以
 * key_board_info_t::cnt 的单位就是毫秒，与 KEY_*_CNT_MAX 的 ms 注释一一对应。
 *
 * 注意：value 只在收到遥控帧时更新（DT7/VT13 都是 14ms 一帧），所以按下/松手
 * 最多晚一帧被感知；边沿状态（RELEASE_TO_PRESS / PRESS_TO_RELEASE）只维持
 * 1 个调用周期，应用若不 1ms 轮询会漏掉。
 *
 * 状态结果用 remote.h 里的 KEY_IS_* 读取，例如：
 *   rc_keyboard_update(sensor->info);
 *   if (KEY_IS_LONG_PRESS(&sensor->info->W)) { ... }
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
