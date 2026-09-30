/* drivers/remote/rc_thread.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/remote/rc_common.h>
#include <drivers/remote/rc_thread.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(rc_thread);

K_SEM_DEFINE(rc_parse_sem, 0, 1);

/* 注册表：只在 APPLICATION 级 SYS_INIT 启动线程之前被改写，之后只读 */
static rc_parser_t* rc_parsers;
static bool rc_thread_running;

/**
 *	@brief	一帧通过终检后的公共处理
 *
 * 顺序与原 dt7_remote.c 的 rc_sensor_update 完全一致：
 * 解析 → 按键状态机 → 数据校验 → 鼠标滤波 → 置在线 → 用户回调
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
    /* 字节的到达时刻只有 ISR 知道，所以空隙判定放在 rc_parser_feed()，
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
    for (rc_parser_t* it = rc_parsers; it != NULL; it = it->next) {
      rc_parser_drain(it);
    }
  }
}

int rc_parser_register(rc_parser_t* parser, const rc_parser_ops_t* ops,
                       const struct device* dev, rc_sensor_t* sensor) {
  if (rc_thread_running) {
    LOG_ERR("parse thread already started, register from POST_KERNEL init");
    return -EBUSY;
  }
  if (parser->ops != NULL) {
    return -EALREADY;
  }
  if (ops == NULL || ops->parse == NULL || ops->frame_size == 0 ||
      ops->frame_size > RC_FRAME_MAX_SIZE) {
    LOG_ERR("invalid parser ops");
    return -EINVAL;
  }

  parser->ops = ops;
  parser->dev = dev;
  parser->sensor = sensor;
  parser->cb = NULL;
  parser->user_data = NULL;
  parser->frame_idx = 0;
  parser->gap_pending = false;
  parser->last_feed_ms = k_uptime_get_32();

  ring_buf_init(&parser->rx_ring, sizeof(parser->rx_ring_storage),
                parser->rx_ring_storage);

  /* 头插。注册窗口内只有这一个执行流，不需要加锁 */
  parser->next = rc_parsers;
  rc_parsers = parser;

  LOG_INF("remote parser registered: frame_size=%u", ops->frame_size);
  return 0;
}

void rc_parser_set_data_ready_cb(rc_parser_t* parser,
                                 remote_data_ready_cb_t cb, void* user_data) {
  parser->cb = cb;
  parser->user_data = user_data;
}

void rc_parser_feed(rc_parser_t* parser, const uint8_t* buf, size_t len) {
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

void rc_parser_reset(rc_parser_t* parser) {
  parser->frame_idx = 0;
  parser->gap_pending = false;
  parser->last_feed_ms = k_uptime_get_32();
  ring_buf_reset(&parser->rx_ring);
}

K_THREAD_STACK_DEFINE(rc_parse_stack, CONFIG_REMOTE_PARSE_THREAD_STACK_SIZE);
static struct k_thread rc_parse_thread_data;

static int rc_thread_start(void) {
  /* APPLICATION 级：所有驱动的 POST_KERNEL init 都已跑完，
   * parser 链表已定型，之后不再被改写 */
  rc_thread_running = true;
  (void)k_thread_create(&rc_parse_thread_data, rc_parse_stack,
                        K_THREAD_STACK_SIZEOF(rc_parse_stack), rc_parse_thread_fn,
                        NULL, NULL, NULL, CONFIG_REMOTE_PARSE_THREAD_PRIO, 0,
                        K_NO_WAIT);
  (void)k_thread_name_set(&rc_parse_thread_data, "rc_parse");
  return 0;
}

SYS_INIT(rc_thread_start, APPLICATION, 0);
