/* breeze/include/drivers/remote/rc_thread.h */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_REMOTE_RC_THREAD_H_
#define DRIVERS_REMOTE_RC_THREAD_H_

#include <drivers/remote/remote.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 间隙法组帧阈值：距上一批字节超过该毫秒数即判定为新帧起点 */
#define RC_FRAME_GAP_MS 5

/** 当前最长的帧（VT13 21B）；DT7 是 18B */
#define RC_FRAME_MAX_SIZE 21

/** ISR → 解析线程 的单实例环形缓冲容量 */
#define RC_RX_RING_SIZE 256

/** 一帧的终检结果 */
typedef enum {
  RC_FRAME_OK = 0, /**< 通过，交给 parse */
  RC_FRAME_DROP,   /**< 帧头/CRC16 不通过，整帧丢弃 */
} rc_frame_check_t;

/**
 * @brief	协议差异描述
 *
 * 组帧（间隙法）、按键状态机、数据校验、鼠标滤波都在公共层做，
 * 驱动只需要提供这三项。
 */
typedef struct {
  uint8_t frame_size; /**< DT7 = 18，VT13 = 21 */

  /**
   * 整帧终检，可为 NULL。
   *
   * DT7 无帧头无 CRC，只能靠间隙法，置 NULL 即可；
   * VT13 在这里校验帧头 + CRC16。返回 RC_FRAME_DROP 时该帧被丢弃，
   * 不会调用 parse，也不会计入解析统计。
   */
  rc_frame_check_t (*frame_check)(const uint8_t* frame, uint8_t len);

  /** 原地解析：只写本帧携带的信号量，跨帧状态原样保留 */
  void (*parse)(const uint8_t* frame, rc_sensor_info_t* info);
} rc_parser_ops_t;

/**
 * @brief	一个遥控实例的组帧上下文，由驱动的 data 结构体持有
 */
typedef struct rc_parser {
  const rc_parser_ops_t* ops; /**< 协议描述 */
  const struct device* dev;   /**< 用户回调的第一个参数，由驱动填 */
  rc_sensor_t* sensor;

  remote_data_ready_cb_t cb; /**< 由驱动注册，在解析线程上下文调用 */
  void* user_data;

  /* --- 组帧状态：只有解析线程访问 --- */
  uint8_t frame[RC_FRAME_MAX_SIZE];
  uint8_t frame_idx;
  /** ISR 检测到字节间空隙，线程在消费后续字节前先据此丢弃半帧 */
  volatile bool gap_pending;

  /* --- ISR 写 / 线程读，SPSC 无锁 --- */
  uint32_t last_feed_ms; /**< 仅 ISR 读写，用于判定空隙 */
  struct ring_buf rx_ring;
  uint8_t rx_ring_storage[RC_RX_RING_SIZE] __aligned(4);

  struct rc_parser* next;
} rc_parser_t;

/**
 * @brief	注册一个遥控实例
 *
 * 必须在解析线程启动前调用。驱动是 POST_KERNEL init，线程是 APPLICATION
 * 级 SYS_INIT 启动，天然满足。
 *
 * @retval 0		成功
 * @retval -EALREADY	该 parser 已注册过
 * @retval -EBUSY	解析线程已启动，注册窗口已关闭
 * @retval -EINVAL	ops 非法
 */
int rc_parser_register(rc_parser_t* parser, const rc_parser_ops_t* ops,
                       const struct device* dev, rc_sensor_t* sensor);

/**
 * @brief	注册数据就绪回调
 *
 * 回调在**解析线程上下文**执行（不再是中断上下文），内部可以做耗时操作。
 *
 * @note	原 DT7 实现在 UART 中断里直接调回调，这是本次行为变化之一。
 */
void rc_parser_set_data_ready_cb(rc_parser_t* parser,
                                 remote_data_ready_cb_t cb, void* user_data);

/**
 * @brief	把收到的字节推进环形缓冲并唤醒解析线程
 *
 * UART 回调（中断上下文）调用；只做 memcpy + k_sem_give，不解析。
 */
void rc_parser_feed(rc_parser_t* parser, const uint8_t* buf, size_t len);

/**
 * @brief	丢弃半帧、清空环形缓冲
 *
 * UART 异常恢复路径（UART_RX_DISABLED）调用，避免半帧残留和后续字节
 * 拼出一个假帧。
 */
void rc_parser_reset(rc_parser_t* parser);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_REMOTE_RC_THREAD_H_ */
