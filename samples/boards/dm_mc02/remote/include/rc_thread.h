/* samples/boards/dm_mc02/remote/include/rc_thread.h */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 *
 * 组帧 + 解析线程，属于应用职责，不放在驱动里。
 * 驱动只负责把 UART 收到的字节交出来（remote_set_rx_cb）并说明本协议的帧格式
 * （remote_get_parser_ops），字节怎么拼成帧、在哪个线程解析由这里决定。
 */

#ifndef RC_THREAD_H_
#define RC_THREAD_H_

#include <drivers/remote/remote.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	把一路遥控设备接到组帧 + 解析线程上
 *
 * 做三件事：从驱动取 sensor 和本协议的 rc_parser_ops_t、给这路设备建组帧上下文
 * （环形缓冲 + 帧状态）、挂上 RX 字节回调。
 *
 * 解析线程在 K_THREAD_DEFINE 里就起来了，平时阻塞在信号量上，不需要另外启动。
 * 注意 attach 之前驱动收到的字节会被丢掉，所以越早调用越好。
 *
 * @retval 0		成功
 * @retval -EALREADY	该设备已经 attach 过
 * @retval -ENODEV	设备没就绪、没有 remote API，或者驱动给不出协议描述
 * @retval -ENOMEM	组帧上下文槽位用完了（见 RC_MAX_PARSERS）
 */
int rc_parse_attach(const struct device* dev);

/**
 * @brief	注册数据就绪回调
 *
 * 回调在**解析线程上下文**执行（不再是中断上下文），内部可以做耗时操作。
 *
 * @retval 0		成功
 * @retval -ENODEV	该设备还没 attach
 */
int rc_parse_set_data_ready_cb(const struct device* dev,
                               remote_data_ready_cb_t cb, void* user_data);

#ifdef __cplusplus
}
#endif

#endif /* RC_THREAD_H_ */
