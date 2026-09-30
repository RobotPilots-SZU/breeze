/* drivers/remote/vt13_remote.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rp_vt13_remote

#include <drivers/remote/rc_common.h>
#include <drivers/remote/remote.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(vt13_remote);

/* 手册《数据帧结构表》：921600 8N1，14ms 一帧 21 字节 */
#define VT13_PACKET_SIZE 21
#define VT13_SOF0 0xA9
#define VT13_SOF1 0x53
#define VT13_CH_OFFSET 1024
#define VT13_CRC_LEN (VT13_PACKET_SIZE - 2)

/* 921600 下一个 DMA 缓冲可能装到上百字节，给足 2 帧 + 余量 */
#define VT13_DMA_BUF_SIZE 64

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
  uint8_t dma_buf[2][VT13_DMA_BUF_SIZE] __aligned(32);
  uint8_t dma_buf_idx;
  /* 驱动不做组帧，收到的字节原样交给应用 */
  remote_rx_cb_t rx_cb;
  void* rx_user_data;
  struct k_work_delayable heartbeat_work;
};

/* -------------------------------------------------------------------------
 * CRC
 * ------------------------------------------------------------------------- */

/**
 *	@brief	CRC-16/MCRF4XX
 *
 * poly 0x8408（0x1021 的反射形式），init 0xFFFF，无输出异或。
 * 与 DJI 官方 crc.c 的 Verify_CRC16_Check_Sum 等价——那张表就是本算法的
 * 查表形式（wCRC_Table[1] == 0x1189 是判据）。手册文字写的是
 * "CRC-16/CCITT-FALSE，无输入输出反转"，两者对同一串数据结果不同；
 * 以官方代码为准，上机用真实抓到的帧复核。
 */
static uint16_t vt13_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
    }
  }
  return crc;
}

/**
 *	@brief	整帧终检：帧头 + CRC16
 *
 * CRC 字段小端存放在最后两字节，校验范围是前 19 字节。
 */
static rc_frame_check_t vt13_frame_check(const uint8_t* frame, uint8_t len) {
  if (len != VT13_PACKET_SIZE || frame[0] != VT13_SOF0 ||
      frame[1] != VT13_SOF1) {
    return RC_FRAME_DROP;
  }

  uint16_t expected =
      (uint16_t)frame[VT13_CRC_LEN] | ((uint16_t)frame[VT13_CRC_LEN + 1] << 8);

  return (vt13_crc16(frame, VT13_CRC_LEN) == expected) ? RC_FRAME_OK
                                                       : RC_FRAME_DROP;
}

/* -------------------------------------------------------------------------
 * 解包
 * ------------------------------------------------------------------------- */

/** 11bit 位域取值并去掉中位偏移，得到 ±660（与 DBUS 同值域） */
static inline int16_t vt13_channel(uint16_t raw) {
  return (int16_t)(raw & 0x07FF) - VT13_CH_OFFSET;
}

/**
 *	@brief	VT13 帧解析（原地写回）
 *
 * 位布局按手册《数据帧结构表》逐位核对，与官方参考实现一致。
 * 只覆盖本帧携带的信号量，跨帧状态由应用侧的组帧层 / rc_common 保留。
 */
static void vt13_data_parse(const uint8_t* buf, rc_sensor_info_t* info) {
  /* 摇杆通道 */
  info->ch0 = vt13_channel((uint16_t)(buf[2] | (buf[3] << 8)));
  info->ch1 = vt13_channel((uint16_t)((buf[3] >> 3) | (buf[4] << 5)));
  info->ch2 = vt13_channel((uint16_t)((buf[4] >> 6) | (buf[5] << 2) |
                                      (buf[6] << 10)));
  info->ch3 = vt13_channel((uint16_t)((buf[6] >> 1) | (buf[7] << 7)));

  /* 挡位切换开关：原始 0/1/2 = C/N/S，原样存 mode_switch。
   *
   * 不写进 s1：breeze 的档位枚举是 RC_SW_UP=1 / MID=3 / DOWN=2，
   * 与 VT13 的 0/1/2 只在中位/下位巧合对应一半，直接映射会得到
   * "1 变 UP、2 变 DOWN、3 永远等不到" 这种半对半错的结果，更难排查。
   * 消费者请读 mode_switch。s1/s2 保持 rc_reset_data 写入的 RC_SW_MID。
   */
  info->mode_switch = (buf[7] >> 4) & 0x03;

  /* VT13 专有按键 */
  info->stop = (buf[7] >> 6) & 0x01;
  info->left_button = (buf[7] >> 7) & 0x01;
  info->right_button = buf[8] & 0x01;
  info->shutter = (buf[9] >> 4) & 0x01;

  /* 拨轮 */
  info->thumbwheel.value = vt13_channel((uint16_t)((buf[8] >> 1) | (buf[9] << 7)));

  /* 鼠标速度：有符号小端 16bit */
  info->mouse_vx = (int16_t)(buf[10] | (buf[11] << 8));
  info->mouse_vy = (int16_t)(buf[12] | (buf[13] << 8));
  info->mouse_vz = (int16_t)(buf[14] | (buf[15] << 8));

  /* 鼠标三键：手册声明各占 2bit，但值域只有 0/1。
   * 只取低位，与 DT7 的 & 0x01 对齐——key_board_info_t 的状态机只认 0/1，
   * 原样透传 2bit 会得到 2/3，switch 匹配不到、状态机静默不动。
   */
  info->mouse_btn_l.value = buf[16] & 0x01;
  info->mouse_btn_r.value = (buf[16] >> 2) & 0x01;
  info->mouse_btn_m.value = (buf[16] >> 4) & 0x01;

  info->key_v = (uint16_t)(buf[17] | (buf[18] << 8));

  /* 键位映射与 breeze 的 KEY_PRESSED_OFFSET_* 逐位一致，无需转换 */
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

  /* 时间戳 */
  info->offline_cnt = 0;
  info->tt1 = info->tt2;
  info->tt2 = k_cyc_to_us_floor32(k_cycle_get_32());
  info->ttp = info->tt2 - info->tt1;
}

static const rc_parser_ops_t vt13_parser_ops = {
    .frame_size = VT13_PACKET_SIZE,
    .frame_check = vt13_frame_check,
    .parse = vt13_data_parse,
};

/* -------------------------------------------------------------------------
 * 驱动框架
 * ------------------------------------------------------------------------- */

static void rc_heartbeat_handler(struct k_work* work) {
  struct k_work_delayable* dwork = k_work_delayable_from_work(work);
  struct rc_sensor_data* data =
      CONTAINER_OF(dwork, struct rc_sensor_data, heartbeat_work);
  rc_sensor_heart_beat(&data->sensor);
  k_work_reschedule(dwork, K_MSEC(14));
}

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
      /* 不做任何组帧：字节原样交给应用，拼帧/CRC 终检/解析都在解析线程里 */
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

  if (!device_is_ready(cfg->uart)) {
    LOG_ERR("UART device not ready");
    return -ENODEV;
  }

  LOG_INF("Initializing VT13 remote sensor");
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
  /* 阈值是配置，只在初始化时写一次；解析是原地写，不会把它们冲掉 */
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

  LOG_INF("VT13 UART initialized on %s", cfg->uart->name);
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
  return &vt13_parser_ops;
}

static const struct remote_driver_api remote_sensor_api = {
    .get_sensor = rc_get_sensor,
    .set_rx_cb = rc_set_rx_cb,
    .get_parser_ops = rc_get_parser_ops,
};

#define VT13_REMOTE_INIT(inst)                                     \
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

DT_INST_FOREACH_STATUS_OKAY(VT13_REMOTE_INIT);
