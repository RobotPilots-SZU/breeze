/* drivers/remote/dt7_remote.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rp_dt7_remote

#include <drivers/remote/rc_common.h>
#include <drivers/remote/remote.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(dt7_remote);

#define DT7_PACKET_SIZE 18
#define DEFAULT_TW_OFFSET 650
#define DEFAULT_TW_MOUSE_OFFSET 400
#define DEFAULT_OFFLINE_CNT 60

struct rc_sensor_cfg {
  const struct device* uart;
  int16_t tw_up_step;
  int16_t tw_down_step;
  int16_t tw_mouseup_step;
  int16_t tw_mousedown_step;
  int16_t offline_max_cnt;
};

struct rc_sensor_data {
  rc_sensor_t sensor;
  rc_sensor_info_t info;
  uint8_t dma_buf[2][36] __aligned(32);
  uint8_t dma_buf_idx;
  /* 驱动不做组帧，收到的字节原样交给应用 */
  remote_rx_cb_t rx_cb;
  void* rx_user_data;
  struct k_work_delayable heartbeat_work;
};

static void rc_heartbeat_handler(struct k_work* work) {
  struct k_work_delayable* dwork = k_work_delayable_from_work(work);
  struct rc_sensor_data* data =
      CONTAINER_OF(dwork, struct rc_sensor_data, heartbeat_work);
  rc_sensor_heart_beat(&data->sensor);
  k_work_reschedule(dwork, K_MSEC(14));
}

/**
 *	@brief	DT7 帧解析（原地写回）
 *
 * 只覆盖本帧携带的信号量。跨帧状态——thumbwheel.value_last / step[]、
 * 鼠标滤波窗口、tw_step_value、offline_max_cnt——原样保留，
 * 因此不需要再靠 memcpy 把配置字段搬回去。
 */
static void rc_data_parse(const uint8_t* rx_buf, rc_sensor_info_t* info) {
  /* Remote channels */
  info->ch0 = (rx_buf[0] | rx_buf[1] << 8) & 0x07FF;
  info->ch0 -= 1024;
  info->ch1 = (rx_buf[1] >> 3 | rx_buf[2] << 5) & 0x07FF;
  info->ch1 -= 1024;
  info->ch2 = (rx_buf[2] >> 6 | rx_buf[3] << 2 | rx_buf[4] << 10) & 0x07FF;
  info->ch2 -= 1024;
  info->ch3 = (rx_buf[4] >> 1 | rx_buf[5] << 7) & 0x07FF;
  info->ch3 -= 1024;

  /* Thumbwheel */
  info->thumbwheel.value =
      ((int16_t)rx_buf[16] | ((int16_t)rx_buf[17] << 8)) & 0x07ff;
  info->thumbwheel.value -= 1024;

  /* Switches */
  info->s1 = ((rx_buf[5] >> 4) & 0x000C) >> 2;
  info->s2 = (rx_buf[5] >> 4) & 0x0003;

  /* Mouse */
  info->mouse_vx = rx_buf[6] | (rx_buf[7] << 8);
  info->mouse_vy = rx_buf[8] | (rx_buf[9] << 8);
  info->mouse_vz = rx_buf[10] | (rx_buf[11] << 8);
  info->mouse_btn_l.value = rx_buf[12] & 0x01;
  info->mouse_btn_r.value = rx_buf[13] & 0x01;
  info->key_v = rx_buf[14] | (rx_buf[15] << 8);

  /* Key states */
  info->W.value = KEY_PRESSED_W(info);
  info->S.value = KEY_PRESSED_S(info);
  info->A.value = KEY_PRESSED_A(info);
  info->D.value = KEY_PRESSED_D(info);
  info->Shift.value = KEY_PRESSED_SHIFT(info);
  info->Ctrl.value = KEY_PRESSED_CTRL(info);
  info->Q.value = KEY_PRESSED_Q(info);
  info->E.value = KEY_PRESSED_E(info);
  info->R.value = KEY_PRESSED_R(info);
  info->F.value = KEY_PRESSED_F(info);
  info->G.value = KEY_PRESSED_G(info);
  info->Z.value = KEY_PRESSED_Z(info);
  info->X.value = KEY_PRESSED_X(info);
  info->C.value = KEY_PRESSED_C(info);
  info->V.value = KEY_PRESSED_V(info);
  info->B.value = KEY_PRESSED_B(info);

  /* Timestamps */
  info->offline_cnt = 0;
  info->tt1 = info->tt2;
  info->tt2 = k_cyc_to_us_floor32(k_cycle_get_32());
  info->ttp = info->tt2 - info->tt1;
}

/**
 * DT7/DBUS 既无帧头也无 CRC，应用侧的组帧只能靠间隙法，
 * 因此 frame_check 置 NULL —— 帧的合法性由 rc_sensor_check 的 ±660 限幅兜底。
 */
static const rc_parser_ops_t dt7_parser_ops = {
    .frame_size = DT7_PACKET_SIZE,
    .frame_check = NULL,
    .parse = rc_data_parse,
};

/** 把 RX 事件转给应用；应用还没挂回调时直接丢 */
static inline void rc_emit_rx(const struct device* dev, remote_rx_event_t ev,
                              const uint8_t* buf, size_t len) {
  struct rc_sensor_data* data = dev->data;
  if (data->rx_cb != NULL) {
    data->rx_cb(dev, ev, buf, len, data->rx_user_data);
  }
}

static void uart_callback(const struct device* uart_dev,
                          struct uart_event* event, void* user_data) {
  const struct device* dev = (const struct device*)user_data;
  struct rc_sensor_data* data = dev->data;
  const struct rc_sensor_cfg* cfg = dev->config;

  switch (event->type) {
    case UART_RX_RDY: {
      uint32_t len = event->data.rx.len;
      uint8_t* chunk = event->data.rx.buf + event->data.rx.offset;

      LOG_HEXDUMP_DBG(chunk, len, "cur_chunk");
      /* 不做任何组帧：字节原样交给应用，拼帧/校验/解析都在解析线程里 */
      rc_emit_rx(dev, REMOTE_RX_DATA, chunk, len);
      break;
    }

    case UART_RX_BUF_REQUEST: {
      data->dma_buf_idx = (data->dma_buf_idx + 1) % 2;
      LOG_DBG("BUF_REQ: swapping to dma_buf[%d]", data->dma_buf_idx);
      uart_rx_buf_rsp(cfg->uart, data->dma_buf[data->dma_buf_idx],
                      sizeof(data->dma_buf[data->dma_buf_idx]));
      break;
    }

    case UART_RX_DISABLED:
      LOG_WRN("RX_DISABLED: re-enabling UART RX");
      data->dma_buf_idx = 0;
      /* 让应用丢掉半帧和环形缓冲里的残留，避免拼出假帧 */
      rc_emit_rx(dev, REMOTE_RX_RESET, NULL, 0);
      uart_rx_enable(cfg->uart, data->dma_buf[0], sizeof(data->dma_buf[0]),
                     1000);
      break;

    case UART_RX_BUF_RELEASED:
      LOG_DBG("RX_BUF_RELEASED: buffer returned by driver");
      break;

    case UART_RX_STOPPED:
      LOG_ERR("RX_STOPPED: reason %d, aborting DMA to recover",
              event->data.rx_stop.reason);
      uart_rx_disable(cfg->uart);
      break;

    default:
      LOG_WRN("Undefined UART behavior: %d", event->type);
      break;
  }
}

static int rc_sensor_init(const struct device* dev) {
  struct rc_sensor_data* data = dev->data;
  const struct rc_sensor_cfg* cfg = dev->config;

  /* Check if UART is ready */
  if (!device_is_ready(cfg->uart)) {
    LOG_ERR("UART device not ready");
    return -ENODEV;
  }

  LOG_INF("Initializing DT7 remote sensor");
  rc_sensor_t* sensor = &data->sensor;
  sensor->info = &data->info;
  sensor->is_online = false;
  sensor->err = NORMAL;

  data->info.tw_step_value[0] = cfg->tw_up_step;
  data->info.tw_step_value[1] = cfg->tw_down_step;
  data->info.tw_step_value[2] = cfg->tw_mouseup_step;
  data->info.tw_step_value[3] = cfg->tw_mousedown_step;
  data->info.offline_max_cnt = cfg->offline_max_cnt;
  data->info.offline_cnt = data->info.offline_max_cnt + 1;

  rc_reset_data(sensor);
  /* 阈值是配置，只在初始化时写一次；解析改成原地后不会再把它们冲掉 */
  rc_keyboard_cnt_max_set(sensor);

  /* 应用挂上 rx_cb 之前的字节会被丢掉，这是有意的：
   * 驱动不缓存，也不替应用决定帧从哪开始。 */
  data->rx_cb = NULL;
  data->rx_user_data = NULL;

  int ret = uart_callback_set(cfg->uart, uart_callback, (void*)dev);
  if (ret < 0) {
    LOG_ERR("Failed to set UART callback: %d", ret);
    sensor->err = DEV_INIT_ERR;
    return ret;
  }

  data->dma_buf_idx = 0;
  ret = uart_rx_enable(cfg->uart, data->dma_buf[0], sizeof(data->dma_buf[0]),
                       1000);
  if (ret < 0) {
    LOG_ERR("Failed to enable UART RX: %d", ret);
    sensor->err = DEV_INIT_ERR;
    return ret;
  }

  k_work_init_delayable(&data->heartbeat_work, rc_heartbeat_handler);
  k_work_reschedule(&data->heartbeat_work, K_MSEC(14));

  LOG_INF("DT7 UART initialized on %s", cfg->uart->name);
  return 0;
}

static rc_sensor_t* rc_get_sensor(const struct device* dev) {
  struct rc_sensor_data* data = dev->data;
  return &data->sensor;
}

static void rc_set_rx_cb(const struct device* dev, remote_rx_cb_t cb,
                         void* user_data) {
  struct rc_sensor_data* data = dev->data;
  data->rx_cb = cb;
  data->rx_user_data = user_data;
}

static const rc_parser_ops_t* rc_get_parser_ops(const struct device* dev) {
  ARG_UNUSED(dev);
  return &dt7_parser_ops;
}

static const struct remote_driver_api remote_sensor_api = {
    .get_sensor = rc_get_sensor,
    .set_rx_cb = rc_set_rx_cb,
    .get_parser_ops = rc_get_parser_ops,
};

/**
 * @brief 检查给定通道的值是否在死区内
 */
static bool rc_is_death_zone(int16_t value, int16_t center, int16_t threshold) {
  return !(value > center + threshold || value < center - threshold);
}

static bool __attribute__((unused)) rc_is_channel_reset(rc_sensor_info_t *info)
{
	return ((!rc_is_death_zone(info->ch0, 0, 50)) &&
          (!rc_is_death_zone(info->ch1, 0, 50)) &&
		      (!rc_is_death_zone(info->ch2, 0, 50)) &&
          (!rc_is_death_zone(info->ch3, 0, 50)));
}

#define DT7_REMOTE_INIT(inst)                                     \
  static struct rc_sensor_data remote_sensor_##inst##_data;        \
  static const struct rc_sensor_cfg remote_sensor_##inst##_cfg = { \
      .uart = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart)),          \
      .tw_up_step = -DEFAULT_TW_OFFSET,                            \
      .tw_down_step = DEFAULT_TW_OFFSET,                           \
      .tw_mouseup_step = -DEFAULT_TW_MOUSE_OFFSET,                 \
      .tw_mousedown_step = DEFAULT_TW_MOUSE_OFFSET,                \
      .offline_max_cnt = DEFAULT_OFFLINE_CNT,                      \
  };                                                               \
  DEVICE_DT_INST_DEFINE(inst, rc_sensor_init, NULL,                \
                        &remote_sensor_##inst##_data,              \
                        &remote_sensor_##inst##_cfg, POST_KERNEL,  \
                        CONFIG_REMOTE_INIT_PRIORITY, &remote_sensor_api)

DT_INST_FOREACH_STATUS_OKAY(DT7_REMOTE_INIT);
