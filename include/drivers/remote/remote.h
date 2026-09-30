/* breeze/include/drivers/remote/remote.h */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_REMOTE_H_
#define DRIVERS_REMOTE_H_

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------- RC Channel Definition------------------------------*/

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH_VALUE_SIDE_WIDTH ((RC_CH_VALUE_MAX - RC_CH_VALUE_MIN) / 2)

/* ----------------------- RC Switch Definition-------------------------------*/

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

/* ----------------------- RC Thumbwheel
 * Definition-------------------------------*/

#define RC_TB_UP ((uint16_t)0)
#define RC_TB_MU ((uint16_t)1)
#define RC_TB_MD ((uint16_t)3)
#define RC_TB_DN ((uint16_t)2)

#define WHEEL_JUMP_VALUE (550)  // 旋钮跳变判断值

/* ----------------------- Key Definition-------------------------------- */

#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01 << 15)

/* 检测按键长按时间 */
#define MOUSE_BTN_L_CNT_MAX 500  // ms 鼠标左键
#define MOUSE_BTN_R_CNT_MAX 500  // ms 鼠标右键
#define MOUSE_BTN_M_CNT_MAX 500  // ms 鼠标中键
#define KEY_Q_CNT_MAX 500        // ms Q键
#define KEY_W_CNT_MAX 1000       // ms W键
#define KEY_E_CNT_MAX 500        // ms E键
#define KEY_R_CNT_MAX 500        // ms R键
#define KEY_A_CNT_MAX 1000       // ms A键
#define KEY_S_CNT_MAX 1000       // ms S键
#define KEY_D_CNT_MAX 1000       // ms D键
#define KEY_F_CNT_MAX 500        // ms F键
#define KEY_G_CNT_MAX 500        // ms G键
#define KEY_Z_CNT_MAX 500        // ms Z键
#define KEY_X_CNT_MAX 500        // ms X键
#define KEY_C_CNT_MAX 500        // ms C键
#define KEY_V_CNT_MAX 500        // ms V键
#define KEY_B_CNT_MAX 500        // ms B键
#define KEY_SHIFT_CNT_MAX 500    // ms SHIFT键
#define KEY_CTRL_CNT_MAX 2500    // ms CTRL键

/* 平滑滤波次数 */
#define REMOTE_SMOOTH_TIMES 10  // 鼠标平滑滤波次数

/* ----------------------- Macro Helpers (accepts rc_sensor_info_t ptr)
 * -------------------------------- */
#define REMOTE_SW1_VALUE(p) ((p)->s1)
#define REMOTE_SW2_VALUE(p) ((p)->s2)
#define REMOTE_LEFT_CH_LR_VALUE(p) ((p)->ch2)
#define REMOTE_LEFT_CH_UD_VALUE(p) ((p)->ch3)
#define REMOTE_RIGH_CH_LR_VALUE(p) ((p)->ch0)
#define REMOTE_RIGH_CH_UD_VALUE(p) ((p)->ch1)
#define REMOTE_THUMB_WHEEL_VALUE(p) ((p)->thumbwheel.value)

#define REMOTE_SW1_UP(p) ((p)->s1 == RC_SW_UP)
#define REMOTE_SW1_MID(p) ((p)->s1 == RC_SW_MID)
#define REMOTE_SW1_DOWN(p) ((p)->s1 == RC_SW_DOWN)
#define REMOTE_SW2_UP(p) ((p)->s2 == RC_SW_UP)
#define REMOTE_SW2_MID(p) ((p)->s2 == RC_SW_MID)
#define REMOTE_SW2_DOWN(p) ((p)->s2 == RC_SW_DOWN)

#define MOUSE_X_MOVE_SPEED(p) ((p)->mouse_vx)
#define MOUSE_Y_MOVE_SPEED(p) ((p)->mouse_vy)
#define MOUSE_Z_MOVE_SPEED(p) ((p)->mouse_vz)
#define MOUSE_PRESSED_LEFT(p) ((p)->mouse_btn_l == 1)
#define MOUSE_PRESSED_RIGHT(p) ((p)->mouse_btn_r == 1)
#define KEY_PRESSED(p) ((p)->key_v)
#define KEY_PRESSED_W(p) (((p)->key_v & KEY_PRESSED_OFFSET_W) != 0)
#define KEY_PRESSED_S(p) (((p)->key_v & KEY_PRESSED_OFFSET_S) != 0)
#define KEY_PRESSED_A(p) (((p)->key_v & KEY_PRESSED_OFFSET_A) != 0)
#define KEY_PRESSED_D(p) (((p)->key_v & KEY_PRESSED_OFFSET_D) != 0)
#define KEY_PRESSED_Q(p) (((p)->key_v & KEY_PRESSED_OFFSET_Q) != 0)
#define KEY_PRESSED_E(p) (((p)->key_v & KEY_PRESSED_OFFSET_E) != 0)
#define KEY_PRESSED_G(p) (((p)->key_v & KEY_PRESSED_OFFSET_G) != 0)
#define KEY_PRESSED_X(p) (((p)->key_v & KEY_PRESSED_OFFSET_X) != 0)
#define KEY_PRESSED_Z(p) (((p)->key_v & KEY_PRESSED_OFFSET_Z) != 0)
#define KEY_PRESSED_C(p) (((p)->key_v & KEY_PRESSED_OFFSET_C) != 0)
#define KEY_PRESSED_B(p) (((p)->key_v & KEY_PRESSED_OFFSET_B) != 0)
#define KEY_PRESSED_V(p) (((p)->key_v & KEY_PRESSED_OFFSET_V) != 0)
#define KEY_PRESSED_F(p) (((p)->key_v & KEY_PRESSED_OFFSET_F) != 0)
#define KEY_PRESSED_R(p) (((p)->key_v & KEY_PRESSED_OFFSET_R) != 0)
#define KEY_PRESSED_CTRL(p) (((p)->key_v & KEY_PRESSED_OFFSET_CTRL) != 0)
#define KEY_PRESSED_SHIFT(p) (((p)->key_v & KEY_PRESSED_OFFSET_SHIFT) != 0)

typedef enum {
  NORMAL,        // 正常(无错误)
  DEV_INIT_ERR,  // 设备初始化错误
  DEV_DATA_ERR,  // 设备数据错误
} dev_errno_t;

/* 按键状态枚举 */
typedef enum {
  KEY_BOARD_RELEASE,
  KEY_BOARD_RELEASE_TO_PRESS,  // 下降沿
  KEY_BOARD_SHORT_PRESS,
  KEY_BOARD_LONG_PRESS,
  KEY_BOARD_PRESS_TO_RELEASE,  // 上升沿
} key_board_status_e;

typedef struct {
  uint8_t value;
  key_board_status_e status;
  key_board_status_e last_status;

  int16_t cnt;
  int16_t cnt_max;
} key_board_info_t;

typedef struct {
  int16_t value_last;
  int16_t value;
  uint8_t step[4];
} thumbwheel_info_t;

typedef struct {
  /* 拨轮跳变值 */
  int16_t tw_step_value[4];

  /* 遥控器通道 */
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  uint8_t s1;
  uint8_t s2;
  uint8_t mode_switch;           // 挡位开关：VT13 为 0/1/2(=C/N/S)，DT7 恒 0
  thumbwheel_info_t thumbwheel;  // 拨轮

  /* 键鼠 */
  int16_t mouse_vx;              // 鼠标x轴速度
  int16_t mouse_vy;              // 鼠标y轴速度
  int16_t mouse_vz;              // 鼠标z轴速度
  float mouse_x;                 // 鼠标x轴滤波后速度
  float mouse_y;                 // 鼠标y轴滤波后速度
  float mouse_z;                 // 鼠标z轴滤波后速度
  key_board_info_t mouse_btn_l;  // 鼠标左键
  key_board_info_t mouse_btn_r;  // 鼠标右键
  key_board_info_t mouse_btn_m;  // 鼠标中键：VT13 有，DT7 恒 0
  key_board_info_t Q;
  key_board_info_t W;
  key_board_info_t E;
  key_board_info_t R;
  key_board_info_t A;
  key_board_info_t S;
  key_board_info_t D;
  key_board_info_t F;
  key_board_info_t G;
  key_board_info_t Z;
  key_board_info_t X;
  key_board_info_t C;
  key_board_info_t V;
  key_board_info_t B;
  key_board_info_t Shift;
  key_board_info_t Ctrl;
  uint16_t key_v;

  /* VT13 专有按键（DT7 恒 0） */
  uint8_t stop;          // 暂停按键
  uint8_t left_button;   // 自定义按键(左)
  uint8_t right_button;  // 自定义按键(右)
  uint8_t shutter;       // 扳机键

  /* 跨帧状态：原地解析时必须保留，不能随每帧清零 */
  int16_t mouse_vx_win[REMOTE_SMOOTH_TIMES];  // 鼠标 x 轴均值滤波窗口
  int16_t mouse_vy_win[REMOTE_SMOOTH_TIMES];  // 鼠标 y 轴均值滤波窗口
  uint8_t mouse_win_idx;                      // 滤波窗口写指针
  int16_t thumbwheel_record;                  // 拨轮本次行程峰值（用于跳变判定）

  int16_t offline_cnt;
  int16_t offline_max_cnt;

  // time tickers
  uint32_t tt1, tt2, ttp;
} rc_sensor_info_t;

typedef struct {
  rc_sensor_info_t* info;
  bool is_online;
  dev_errno_t err;
} rc_sensor_t;

/* ----------------------- 协议描述 -------------------------------- */

/** 一帧的终检结果 */
typedef enum {
  RC_FRAME_OK = 0, /**< 通过，交给 parse */
  RC_FRAME_DROP,   /**< 帧头/CRC16 不通过，整帧丢弃 */
} rc_frame_check_t;

/**
 * @brief	协议差异描述
 *
 * 驱动只描述「本协议的帧长什么样、拿到整帧后怎么解」，不参与组帧：
 * 字节怎么拼成帧、拼好的帧什么时候被解析、在哪个线程解析，都是应用的事。
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

/* ----------------------- RX 字节回调 -------------------------------- */

/** RX 事件 */
typedef enum {
  REMOTE_RX_DATA,  /**< 收到一段字节，buf/len 有效 */
  REMOTE_RX_RESET, /**< UART 重启，应用应丢掉半帧和环形缓冲里的残留 */
} remote_rx_event_t;

/**
 * @brief	UART 收到的原始字节回调
 *
 * **中断上下文**调用。驱动不做任何组帧，收到什么就原样交出去什么。
 */
typedef void (*remote_rx_cb_t)(const struct device* dev, remote_rx_event_t ev,
                               const uint8_t* buf, size_t len, void* user_data);

/* ----------------------- Driver API -------------------------------- */

typedef rc_sensor_t* (*remote_api_get_sensor)(const struct device* dev);

typedef void (*remote_data_ready_cb_t)(const struct device* dev,
                                       rc_sensor_t* sensor, void* user_data);

typedef void (*remote_api_set_rx_cb)(const struct device* dev,
                                     remote_rx_cb_t cb, void* user_data);

typedef const rc_parser_ops_t* (*remote_api_get_parser_ops)(
    const struct device* dev);

/**
 * 驱动只做三件事：交出 sensor、把 UART 字节转给 set_rx_cb、说明本协议帧的格式。
 * 组帧和解析线程不在驱动里。
 */
struct remote_driver_api {
  remote_api_get_sensor get_sensor;
  remote_api_set_rx_cb set_rx_cb;
  remote_api_get_parser_ops get_parser_ops;
};

static inline rc_sensor_t* remote_get_sensor(const struct device* dev) {
  const struct remote_driver_api* api =
      (const struct remote_driver_api*)dev->api;
  if (!api || api->get_sensor == NULL) {
    return NULL;
  }
  return api->get_sensor(dev);
}

/**
 * @brief	注册原始字节回调
 *
 * 回调在中断上下文执行，只应该做搬运（例如推进环形缓冲），不要在里面解析。
 * 注册之前的字节会被丢弃。
 */
static inline int remote_set_rx_cb(const struct device* dev,
                                   remote_rx_cb_t cb, void* user_data) {
  const struct remote_driver_api* api =
      (const struct remote_driver_api*)dev->api;
  if (!api || api->set_rx_cb == NULL) {
    return -ENOSYS;
  }
  api->set_rx_cb(dev, cb, user_data);
  return 0;
}

/**
 * @brief	取本协议的帧描述
 *
 * 应用拿到之后用它建组帧上下文（见 samples 里的 rc_thread.c）。
 */
static inline const rc_parser_ops_t* remote_get_parser_ops(
    const struct device* dev) {
  const struct remote_driver_api* api =
      (const struct remote_driver_api*)dev->api;
  if (!api || api->get_parser_ops == NULL) {
    return NULL;
  }
  return api->get_parser_ops(dev);
}

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_REMOTE_H_ */
