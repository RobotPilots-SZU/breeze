/* samples/boards/dm_mc02/remote/src/rc_thread.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rc_thread.h"

#include <drivers/remote/rc_common.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(rc_thread);

/** 间隙法组帧阈值：距上一批字节超过该毫秒数即判定为新帧起点 */
#define RC_FRAME_GAP_MS 5

/** 当前最长的帧（VT13 21B）；DT7 是 18B */
#define RC_FRAME_MAX_SIZE 21

/** ISR → 解析线程 的单实例环形缓冲容量 */
#define RC_RX_RING_SIZE 256

/** 同时能挂几路遥控。DT7 和 VT13 通常只上一路，留两个余量 */
#define RC_MAX_PARSERS 2

/** 一路遥控的组帧上下文 */
typedef struct rc_parser {
  const struct device* dev;    /**< NULL 表示这个槽位空着 */
  const rc_parser_ops_t* ops;  /**< 由驱动经 remote_get_parser_ops() 给出 */
  rc_sensor_t* sensor;

  remote_data_ready_cb_t cb; /**< 应用回调，在解析线程上下文跑 */
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
} rc_parser_t;

K_SEM_DEFINE(rc_parse_sem, 0, 1);

static rc_parser_t rc_parsers[RC_MAX_PARSERS];

static rc_parser_t* rc_parser_find(const struct device* dev) {
  for (size_t i = 0; i < ARRAY_SIZE(rc_parsers); i++) {
    if (rc_parsers[i].dev == dev) {
      return &rc_parsers[i];
    }
  }
  return NULL;
}

/**
 *	@brief	一帧通过终检后的公共处理
 *
 * 顺序与原来的实现一致：
 * 解析 → 按键状态机 → 数据校验 → 鼠标滤波 → 置在线 → 应用回调
 */
static void rc_parser_handle_frame(rc_parser_t* parser) {
  const rc_parser_ops_t* ops = parser->ops;
  rc_sensor_t* sensor = parser->sensor;

  LOG_HEXDUMP_DBG(parser->frame, ops->frame_size, "frame");

  if (ops->frame_check != NULL &&
      ops->frame_check(parser->frame, ops->frame_size) != RC_FRAME_OK) {
    LOG_DBG("frame rejected (%u bytes)", ops->frame_size);
    return;
  }

  ops->parse(parser->frame, sensor->info);

  rc_keyboard_update(sensor->info);
  rc_sensor_check(sensor);
  rc_interrupt_update(sensor);

  sensor->is_online = true;

  if (parser->cb != NULL) {
    parser->cb(parser->dev, sensor, parser->user_data);
  }
}

static void rc_parser_drain(rc_parser_t* parser) {
  uint8_t chunk[32];

  for (;;) {
    /* 字节的到达时刻只有 ISR 知道，所以空隙判定放在 rc_rx_cb()，
     * 它总在推入空隙之后的字节之前置位 gap_pending——这里先看标志再取数据，
     * 就一定能赶在那些字节之前把半帧丢掉。
     */
    if (parser->gap_pending) {
      parser->gap_pending = false;
      if (parser->frame_idx != 0) {
        LOG_DBG("byte gap, drop partial frame (idx=%u)", parser->frame_idx);
        parser->frame_idx = 0;
      }
    }

    uint32_t len = ring_buf_get(&parser->rx_ring, chunk, sizeof(chunk));
    if (len == 0) {
      return;
    }

    for (uint32_t i = 0; i < len; i++) {
      parser->frame[parser->frame_idx++] = chunk[i];
      if (parser->frame_idx == parser->ops->frame_size) {
        rc_parser_handle_frame(parser);
        parser->frame_idx = 0;
      }
    }
  }
}

static void rc_parse_thread_fn(void* p1, void* p2, void* p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&rc_parse_sem, K_FOREVER);
    for (size_t i = 0; i < ARRAY_SIZE(rc_parsers); i++) {
      if (rc_parsers[i].dev != NULL) {
        rc_parser_drain(&rc_parsers[i]);
      }
    }
  }
}

K_THREAD_DEFINE(rc_parse_thread, CONFIG_REMOTE_PARSE_THREAD_STACK_SIZE,
                rc_parse_thread_fn, NULL, NULL, NULL,
                CONFIG_REMOTE_PARSE_THREAD_PRIO, 0, 0);

/**
 *	@brief	丢掉半帧、清空环形缓冲
 *
 * 只在 UART_RX_DISABLED 恢复路径上调用（中断上下文），避免半帧残留和后续字节
 * 拼出一个假帧。
 */
static void rc_parser_reset(rc_parser_t* parser) {
  parser->frame_idx = 0;
  parser->gap_pending = false;
  parser->last_feed_ms = k_uptime_get_32();
  ring_buf_reset(&parser->rx_ring);
}

/**
 *	@brief	驱动交出来的 RX 事件（中断上下文）
 *
 * 只做空隙判定、memcpy 进环形缓冲、唤醒解析线程，不解析。
 */
static void rc_rx_cb(const struct device* dev, remote_rx_event_t ev,
                     const uint8_t* buf, size_t len, void* user_data) {
  rc_parser_t* parser = user_data;

  if (ev == REMOTE_RX_RESET) {
    rc_parser_reset(parser);
    return;
  }

  if (len == 0) {
    return;
  }

  uint32_t now = k_uptime_get_32();
  if ((now - parser->last_feed_ms) >= RC_FRAME_GAP_MS) {
    parser->gap_pending = true;
  }
  parser->last_feed_ms = now;

  uint32_t put = ring_buf_put(&parser->rx_ring, buf, (uint32_t)len);
  if (put < len) {
    /* 解析线程没跟上，尾部字节被丢。gap_pending 会保证下一帧重新对齐 */
    LOG_WRN("rx ring full, dropped %u bytes", (uint32_t)(len - put));
  }

  k_sem_give(&rc_parse_sem);
}

int rc_parse_attach(const struct device* dev) {
  if (rc_parser_find(dev) != NULL) {
    LOG_WRN("device already attached");
    return -EALREADY;
  }

  rc_parser_t* parser = NULL;
  for (size_t i = 0; i < ARRAY_SIZE(rc_parsers); i++) {
    if (rc_parsers[i].dev == NULL) {
      parser = &rc_parsers[i];
      break;
    }
  }
  if (parser == NULL) {
    LOG_ERR("no free parser slot (RC_MAX_PARSERS=%d)", RC_MAX_PARSERS);
    return -ENOMEM;
  }

  rc_sensor_t* sensor = remote_get_sensor(dev);
  const rc_parser_ops_t* ops = remote_get_parser_ops(dev);
  if (sensor == NULL || ops == NULL || ops->parse == NULL ||
      ops->frame_size == 0 || ops->frame_size > RC_FRAME_MAX_SIZE) {
    LOG_ERR("device has no usable sensor / parser ops");
    return -ENODEV;
  }

  parser->ops = ops;
  parser->sensor = sensor;
  parser->cb = NULL;
  parser->user_data = NULL;
  parser->frame_idx = 0;
  parser->gap_pending = false;
  parser->last_feed_ms = k_uptime_get_32();
  ring_buf_init(&parser->rx_ring, sizeof(parser->rx_ring_storage),
                parser->rx_ring_storage);

  /* dev 最后写：解析线程只认 dev != NULL 的槽位，这样它要么跳过、要么看到
   * 一个已经填好的上下文。上一次 attach 的字节早已被 drain 完，
   * 且此刻还没挂 rx_cb，不会有新字节进来。 */
  parser->dev = dev;

  int ret = remote_set_rx_cb(dev, rc_rx_cb, parser);
  if (ret < 0) {
    parser->dev = NULL;
    LOG_ERR("failed to hook rx callback: %d", ret);
    return ret;
  }

  LOG_INF("remote attached: frame_size=%u", ops->frame_size);
  return 0;
}

int rc_parse_set_data_ready_cb(const struct device* dev,
                               remote_data_ready_cb_t cb, void* user_data) {
  rc_parser_t* parser = rc_parser_find(dev);
  if (parser == NULL) {
    LOG_ERR("device not attached, call rc_parse_attach() first");
    return -ENODEV;
  }

  parser->cb = cb;
  parser->user_data = user_data;
  return 0;
}
