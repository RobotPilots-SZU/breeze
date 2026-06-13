/* breeze/include/drivers/remote.h */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_REMOTE_H_
#define DRIVERS_REMOTE_H_

#include <errno.h>
#include <math.h>
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
#define REMOTE_SW1_VALUE(p) (p)((p)->s1)
#define REMOTE_SW2_VALUE(p) (p)((p)->s1)
#define REMOTE_LEFT_CH_LR_VALUE(p) ((p)->ch2)
#define REMOTE_LEFT_CH_UD_VALUE(p) ((p)->ch3)
#define REMOTE_RIGH_CH_LR_VALUE(p) ((p)->ch0)
#define REMOTE_RIGH_CH_UD_VALUE(p) ((p)->ch1)
#define REMOTE_THUMB_WHEEL_VALUE (p)((p)->thumbwheel)

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

/* ----------------------- Driver API -------------------------------- */

typedef rc_sensor_t* (*remote_api_get_sensor)(const struct device* dev);

typedef void (*remote_data_ready_cb_t)(const struct device* dev,
                                       rc_sensor_t* sensor, void* user_data);
typedef void (*remote_api_set_data_ready_cb)(const struct device* dev,
                                             remote_data_ready_cb_t cb,
                                             void* user_data);

struct remote_driver_api {
  remote_api_get_sensor get_sensor;
  remote_api_set_data_ready_cb set_data_ready_cb;
};

static inline rc_sensor_t* remote_get_sensor(const struct device* dev) {
  const struct remote_driver_api* api =
      (const struct remote_driver_api*)dev->api;
  if (!api || api->get_sensor == NULL) {
    return NULL;
  }
  return api->get_sensor(dev);
}

static inline int remote_set_data_ready_cb(const struct device* dev,
                                           remote_data_ready_cb_t cb,
                                           void* user_data) {
  const struct remote_driver_api* api =
      (const struct remote_driver_api*)dev->api;
  if (!api || api->set_data_ready_cb == NULL) {
    return -ENOSYS;
  }
  api->set_data_ready_cb(dev, cb, user_data);
  return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_REMOTE_H_ */
