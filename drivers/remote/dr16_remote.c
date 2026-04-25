/* drivers/remote/dr16_remote.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rp_remote

#include <drivers/remote.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(dr16_remote);

#define DR16_PACKET_SIZE 18
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
  uint8_t frame_buf[DR16_PACKET_SIZE];
  uint8_t frame_idx;
  uint32_t last_rx_time;
  remote_data_ready_cb_t cb;
  void* user_data;
  struct k_work_delayable heartbeat_work;
};

/**
 */
static void rc_keyboard_cnt_max_set(rc_sensor_t* sensor) {
  sensor->info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
  sensor->info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;
  sensor->info->Q.cnt_max = KEY_Q_CNT_MAX;
  sensor->info->W.cnt_max = KEY_W_CNT_MAX;
  sensor->info->E.cnt_max = KEY_E_CNT_MAX;
  sensor->info->R.cnt_max = KEY_R_CNT_MAX;
  sensor->info->A.cnt_max = KEY_A_CNT_MAX;
  sensor->info->S.cnt_max = KEY_S_CNT_MAX;
  sensor->info->D.cnt_max = KEY_D_CNT_MAX;
  sensor->info->F.cnt_max = KEY_F_CNT_MAX;
  sensor->info->G.cnt_max = KEY_G_CNT_MAX;
  sensor->info->Z.cnt_max = KEY_Z_CNT_MAX;
  sensor->info->X.cnt_max = KEY_X_CNT_MAX;
  sensor->info->C.cnt_max = KEY_C_CNT_MAX;
  sensor->info->V.cnt_max = KEY_V_CNT_MAX;
  sensor->info->B.cnt_max = KEY_B_CNT_MAX;
  sensor->info->Shift.cnt_max = KEY_SHIFT_CNT_MAX;
  sensor->info->Ctrl.cnt_max = KEY_CTRL_CNT_MAX;
}

/**
 */
static void rc_interrupt_update(rc_sensor_t* sensor) {
  /* 鼠标速度均值滤波 */
  static int16_t mouse_x[REMOTE_SMOOTH_TIMES], mouse_y[REMOTE_SMOOTH_TIMES];
  static int16_t index = 0;

  if (index == REMOTE_SMOOTH_TIMES) {
    index = 0;
  }

  sensor->info->mouse_x -= (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
  sensor->info->mouse_y -= (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;
  mouse_x[index] = sensor->info->mouse_vx;
  mouse_y[index] = sensor->info->mouse_vy;
  sensor->info->mouse_x += (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
  sensor->info->mouse_y += (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;

  index++;
}

/**
 *	@brief	更新键盘按键状态
 *  release -> release_to_press -> short_press -> long_press -> press_to_release
 */
static void rc_keyboard_status_update(key_board_info_t* key) {
  key->last_status = key->status;

  switch (key->value) {
    case 0: {
      if (key->cnt != 0) {
        key->status = KEY_BOARD_PRESS_TO_RELEASE;
        key->cnt = 0;
        LOG_DBG("Key press_to_release");
      } else {
        key->status = KEY_BOARD_RELEASE;
        key->cnt = 0;
      }
      break;
    }
    case 1: {
      key->cnt++;
      if (key->cnt == 1) {
        key->status = KEY_BOARD_RELEASE_TO_PRESS;
        LOG_DBG("Key release_to_press");
      } else if (key->cnt >= key->cnt_max) {
        key->status = KEY_BOARD_LONG_PRESS;
        key->cnt = key->cnt_max;
        LOG_DBG("Key long_press (cnt=%d)", key->cnt_max);
      } else {
        key->status = KEY_BOARD_SHORT_PRESS;
      }
    }
  }
}

static void rc_keyboard_update(rc_sensor_info_t* info) {
  rc_keyboard_status_update(&info->mouse_btn_l);
  rc_keyboard_status_update(&info->mouse_btn_r);
  rc_keyboard_status_update(&info->Q);
  rc_keyboard_status_update(&info->W);
  rc_keyboard_status_update(&info->E);
  rc_keyboard_status_update(&info->R);
  rc_keyboard_status_update(&info->A);
  rc_keyboard_status_update(&info->S);
  rc_keyboard_status_update(&info->D);
  rc_keyboard_status_update(&info->F);
  rc_keyboard_status_update(&info->G);
  rc_keyboard_status_update(&info->Z);
  rc_keyboard_status_update(&info->X);
  rc_keyboard_status_update(&info->C);
  rc_keyboard_status_update(&info->V);
  rc_keyboard_status_update(&info->B);
  rc_keyboard_status_update(&info->Shift);
  rc_keyboard_status_update(&info->Ctrl);
}

static int16_t abs_int16(int16_t x) { return x < 0 ? -x : x; }

static void rc_sensor_check(rc_sensor_t* sensor) {
  static int16_t thumbwheel_record = 0;
  rc_sensor_info_t* info = sensor->info;

  if ((abs_int16(info->thumbwheel.value_last) <
       abs_int16(info->thumbwheel.value)) &&
      (abs_int16(thumbwheel_record) < abs_int16(info->thumbwheel.value))) {
    thumbwheel_record = info->thumbwheel.value;
  }

  if ((info->thumbwheel.value == 0) && (thumbwheel_record != 0)) {
    for (char i = 0; i < 4; i++) {
      if (info->tw_step_value[i] > 0 && thumbwheel_record > 0) {
        if (thumbwheel_record >= info->tw_step_value[i]) {
          info->thumbwheel.step[i] = !info->thumbwheel.step[i];
          LOG_DBG("Thumbwheel step[%d] toggled to %d", i,
                  info->thumbwheel.step[i]);
          thumbwheel_record = 0;
        }
      }
      if (info->tw_step_value[i] < 0 && thumbwheel_record < 0) {
        if (thumbwheel_record <= info->tw_step_value[i]) {
          info->thumbwheel.step[i] = !info->thumbwheel.step[i];
          LOG_DBG("Thumbwheel step[%d] toggled to %d", i,
                  info->thumbwheel.step[i]);
          thumbwheel_record = 0;
        }
      }
    }
    thumbwheel_record = 0;
  }

  info->thumbwheel.value_last = info->thumbwheel.value;

  if (abs_int16(info->ch0) > 660 || abs_int16(info->ch1) > 660 ||
      abs_int16(info->ch2) > 660 || abs_int16(info->ch3) > 660) {
    sensor->err = DEV_DATA_ERR;
    LOG_WRN("DR16 Data Err: ch0=%d ch1=%d ch2=%d ch3=%d", info->ch0, info->ch1,
            info->ch2, info->ch3);
    info->ch0 = 0;
    info->ch1 = 0;
    info->ch2 = 0;
    info->ch3 = 0;
    info->s1 = RC_SW_MID;
    info->s2 = RC_SW_MID;
    info->thumbwheel.value = 0;
    info->thumbwheel.value_last = 0;
    info->thumbwheel.step[RC_TB_UP] = 0;
    info->thumbwheel.step[RC_TB_MU] = 0;
    info->thumbwheel.step[RC_TB_MD] = 0;
    info->thumbwheel.step[RC_TB_DN] = 0;
  } else {
    sensor->err = NORMAL;
  }
}

static void rc_sensor_heart_beat(rc_sensor_t* sensor) {
  rc_sensor_info_t* info = sensor->info;

  info->offline_cnt++;
  if (info->offline_cnt > info->offline_max_cnt) {
    info->offline_cnt = info->offline_max_cnt;
    if (sensor->is_online) {
      // LOG_WRN("Remote went offline");
    }
    sensor->is_online = false;
  } else {
    if (!sensor->is_online) {
      // LOG_INF("Remote back online");
    }
    sensor->is_online = true;
  }
}

static void rc_heartbeat_handler(struct k_work* work) {
  struct k_work_delayable* dwork = k_work_delayable_from_work(work);
  struct rc_sensor_data* data =
      CONTAINER_OF(dwork, struct rc_sensor_data, heartbeat_work);
  rc_sensor_heart_beat(&data->sensor);
  k_work_reschedule(dwork, K_MSEC(14));
}

static void rc_reset_data(rc_sensor_t* sensor) {
  sensor->info->ch0 = 0;
  sensor->info->ch1 = 0;
  sensor->info->ch2 = 0;
  sensor->info->ch3 = 0;
  sensor->info->s1 = RC_SW_MID;
  sensor->info->s2 = RC_SW_MID;
  sensor->info->mouse_vx = 0;
  sensor->info->mouse_vy = 0;
  sensor->info->mouse_vz = 0;
  sensor->info->mouse_x = 0.f;
  sensor->info->mouse_y = 0.f;
  sensor->info->mouse_z = 0.f;
  sensor->info->mouse_btn_l.value = 0;
  sensor->info->mouse_btn_r.value = 0;
  sensor->info->key_v = 0;
  sensor->info->W.value = 0;
  sensor->info->S.value = 0;
  sensor->info->A.value = 0;
  sensor->info->D.value = 0;
  sensor->info->Shift.value = 0;
  sensor->info->Ctrl.value = 0;
  sensor->info->Q.value = 0;
  sensor->info->E.value = 0;
  sensor->info->R.value = 0;
  sensor->info->F.value = 0;
  sensor->info->G.value = 0;
  sensor->info->Z.value = 0;
  sensor->info->X.value = 0;
  sensor->info->C.value = 0;
  sensor->info->V.value = 0;
  sensor->info->B.value = 0;
  sensor->info->thumbwheel.value = 0;
  sensor->info->thumbwheel.value_last = 0;
  sensor->info->thumbwheel.step[RC_TB_UP] = 0;
  sensor->info->thumbwheel.step[RC_TB_MU] = 0;
  sensor->info->thumbwheel.step[RC_TB_MD] = 0;
  sensor->info->thumbwheel.step[RC_TB_DN] = 0;
  sensor->info->tt1 = 0;
  sensor->info->tt2 = 0;
  sensor->info->ttp = 0;
}

/**
 *	@brief	遥控器数据解析协议
 */
static rc_sensor_info_t rc_data_parse(uint8_t* rx_buf) {
  rc_sensor_info_t rc_info = {0};

  /* Remote channels */
  rc_info.ch0 = (rx_buf[0] | rx_buf[1] << 8) & 0x07FF;
  rc_info.ch0 -= 1024;
  rc_info.ch1 = (rx_buf[1] >> 3 | rx_buf[2] << 5) & 0x07FF;
  rc_info.ch1 -= 1024;
  rc_info.ch2 = (rx_buf[2] >> 6 | rx_buf[3] << 2 | rx_buf[4] << 10) & 0x07FF;
  rc_info.ch2 -= 1024;
  rc_info.ch3 = (rx_buf[4] >> 1 | rx_buf[5] << 7) & 0x07FF;
  rc_info.ch3 -= 1024;

  /* Thumbwheel */
  rc_info.thumbwheel.value =
      ((int16_t)rx_buf[16] | ((int16_t)rx_buf[17] << 8)) & 0x07ff;
  rc_info.thumbwheel.value -= 1024;

  /* Switches */
  rc_info.s1 = ((rx_buf[5] >> 4) & 0x000C) >> 2;
  rc_info.s2 = (rx_buf[5] >> 4) & 0x0003;

  /* Mouse */
  rc_info.mouse_vx = rx_buf[6] | (rx_buf[7] << 8);
  rc_info.mouse_vy = rx_buf[8] | (rx_buf[9] << 8);
  rc_info.mouse_vz = rx_buf[10] | (rx_buf[11] << 8);
  rc_info.mouse_btn_l.value = rx_buf[12] & 0x01;
  rc_info.mouse_btn_r.value = rx_buf[13] & 0x01;
  rc_info.key_v = rx_buf[14] | (rx_buf[15] << 8);

  /* Key states */
  rc_info.W.value = KEY_PRESSED_W(&rc_info);
  rc_info.S.value = KEY_PRESSED_S(&rc_info);
  rc_info.A.value = KEY_PRESSED_A(&rc_info);
  rc_info.D.value = KEY_PRESSED_D(&rc_info);
  rc_info.Shift.value = KEY_PRESSED_SHIFT(&rc_info);
  rc_info.Ctrl.value = KEY_PRESSED_CTRL(&rc_info);
  rc_info.Q.value = KEY_PRESSED_Q(&rc_info);
  rc_info.E.value = KEY_PRESSED_E(&rc_info);
  rc_info.R.value = KEY_PRESSED_R(&rc_info);
  rc_info.F.value = KEY_PRESSED_F(&rc_info);
  rc_info.G.value = KEY_PRESSED_G(&rc_info);
  rc_info.Z.value = KEY_PRESSED_Z(&rc_info);
  rc_info.X.value = KEY_PRESSED_X(&rc_info);
  rc_info.C.value = KEY_PRESSED_C(&rc_info);
  rc_info.V.value = KEY_PRESSED_V(&rc_info);
  rc_info.B.value = KEY_PRESSED_B(&rc_info);

  /* Timestamps */
  rc_info.offline_cnt = 0;
  rc_info.tt1 = rc_info.tt2;
  rc_info.tt2 = k_cyc_to_us_floor32(k_cycle_get_32());
  rc_info.ttp = rc_info.tt2 - rc_info.tt1;

  // LOG_INF("Parsed: ch0=%d, ch1=%d, ch2=%d, ch3=%d, s1=%d, s2=%d", rc_info.ch0,
          // rc_info.ch1, rc_info.ch2, rc_info.ch3, rc_info.s1, rc_info.s2);

  return rc_info;
}

static void rc_sensor_update(const struct device* dev, uint8_t* rx_buf);

static void uart_callback(const struct device* uart_dev,
                          struct uart_event* event, void* user_data) {
  const struct device* dev = (const struct device*)user_data;
  struct rc_sensor_data* data = dev->data;
  const struct rc_sensor_cfg* cfg = dev->config;

  switch (event->type) {
    case UART_RX_RDY: {
      uint32_t now = k_uptime_get_32();
      uint32_t len = event->data.rx.len;
      uint8_t* chunk = event->data.rx.buf + event->data.rx.offset;

      LOG_HEXDUMP_DBG(chunk, DR16_PACKET_SIZE, "cur_chunk");
      if (now - data->last_rx_time >= 5) {
        LOG_DBG("Gap detected: %d ms, resetting frame_idx",
                now - data->last_rx_time);
        data->frame_idx = 0;
      }
      data->last_rx_time = now;

      LOG_DBG("RX_RDY: len=%d, frame_idx_before=%d", len, data->frame_idx);

      for (uint32_t i = 0; i < len; i++) {
        if (data->frame_idx < DR16_PACKET_SIZE) {
          data->frame_buf[data->frame_idx++] = chunk[i];
        }
        if (data->frame_idx == DR16_PACKET_SIZE) {
          LOG_HEXDUMP_DBG(data->frame_buf, DR16_PACKET_SIZE, "FULL DR16 FRAME");
          rc_sensor_update(dev, data->frame_buf);
          data->frame_idx = 0;
        }
      }
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
      data->frame_idx = 0;
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

  LOG_INF("Initializing DR16 remote sensor");
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
  rc_keyboard_cnt_max_set(sensor);

  int ret = uart_callback_set(cfg->uart, uart_callback, (void*)dev);
  if (ret < 0) {
    LOG_ERR("Failed to set UART callback: %d", ret);
    sensor->err = DEV_INIT_ERR;
    return ret;
  }

  data->dma_buf_idx = 0;
  data->frame_idx = 0;
  data->last_rx_time = k_uptime_get_32();
  ret = uart_rx_enable(cfg->uart, data->dma_buf[0], sizeof(data->dma_buf[0]),
                       1000);
  if (ret < 0) {
    LOG_ERR("Failed to enable UART RX: %d", ret);
    sensor->err = DEV_INIT_ERR;
    return ret;
  }

  k_work_init_delayable(&data->heartbeat_work, rc_heartbeat_handler);
  k_work_reschedule(&data->heartbeat_work, K_MSEC(14));

  LOG_INF("DR16 UART initialized on %s", cfg->uart->name);
  return 0;
}

/**
 *	@brief	更新遥控数据
 */
static void rc_sensor_update(const struct device* dev, uint8_t* rx_buf) {
  struct rc_sensor_data* data = dev->data;
  rc_sensor_t* sensor = &data->sensor;

  // LOG_HEXDUMP_INF(rx_buf, DR16_PACKET_SIZE, "DR16 RAW FRAME");

  rc_sensor_info_t rc_info_new = rc_data_parse(rx_buf);

  rc_info_new.offline_max_cnt = sensor->info->offline_max_cnt;
  memcpy(rc_info_new.tw_step_value, sensor->info->tw_step_value, sizeof(rc_info_new.tw_step_value));
  memcpy(sensor->info, &rc_info_new, sizeof(rc_sensor_info_t));

  rc_keyboard_cnt_max_set(sensor);
  rc_keyboard_update(sensor->info);
  rc_sensor_check(sensor);
  rc_interrupt_update(sensor);

  sensor->is_online = true;

  if (data->cb) {
    data->cb(dev, sensor, data->user_data);
  }
}

static rc_sensor_t* rc_get_sensor(const struct device* dev) {
  struct rc_sensor_data* data = dev->data;
  return &data->sensor;
}

static void rc_set_data_ready_cb(const struct device* dev,
                                 remote_data_ready_cb_t cb, void* user_data) {
  struct rc_sensor_data* data = dev->data;
  data->cb = cb;
  data->user_data = user_data;
}

static const struct remote_driver_api remote_sensor_api = {
    .get_sensor = rc_get_sensor,
    .set_data_ready_cb = rc_set_data_ready_cb,
};

/**
 * @brief 检查给定通道的值是否在死区内
 */
static bool rc_is_death_zone(int16_t value, int16_t center, int16_t threshold) {
  return !(value > center + threshold || value < center - threshold);
}

static bool rc_is_channel_reset(rc_sensor_info_t* info) {
  return ((!rc_is_death_zone(info->ch0, 0, 50)) &&
          (!rc_is_death_zone(info->ch1, 0, 50)) &&
          (!rc_is_death_zone(info->ch2, 0, 50)) &&
          (!rc_is_death_zone(info->ch3, 0, 50)));
}

#define DR16_REMOTE_INIT(inst)                                     \
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

DT_INST_FOREACH_STATUS_OKAY(DR16_REMOTE_INIT);