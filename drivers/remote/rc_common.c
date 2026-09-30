/* drivers/remote/rc_common.c */
/*
 * Copyright (c) 2026 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/remote/rc_common.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_REMOTE_LOG_LEVEL
LOG_MODULE_REGISTER(rc_common);

void rc_keyboard_cnt_max_set(rc_sensor_t* sensor) {
  sensor->info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
  sensor->info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;
  sensor->info->mouse_btn_m.cnt_max = MOUSE_BTN_M_CNT_MAX;
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

void rc_keyboard_update(rc_sensor_info_t* info) {
  rc_keyboard_status_update(&info->mouse_btn_l);
  rc_keyboard_status_update(&info->mouse_btn_r);
  rc_keyboard_status_update(&info->mouse_btn_m);
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

/**
 *	@brief	鼠标速度均值滤波
 *
 * 滑窗保存在 info 里（而不是 static 局部变量），这样多实例互不干扰。
 */
void rc_interrupt_update(rc_sensor_t* sensor) {
  rc_sensor_info_t* info = sensor->info;
  uint8_t index = info->mouse_win_idx;

  if (index >= REMOTE_SMOOTH_TIMES) {
    index = 0;
  }

  info->mouse_x -= (float)info->mouse_vx_win[index] / (float)REMOTE_SMOOTH_TIMES;
  info->mouse_y -= (float)info->mouse_vy_win[index] / (float)REMOTE_SMOOTH_TIMES;
  info->mouse_vx_win[index] = info->mouse_vx;
  info->mouse_vy_win[index] = info->mouse_vy;
  info->mouse_x += (float)info->mouse_vx_win[index] / (float)REMOTE_SMOOTH_TIMES;
  info->mouse_y += (float)info->mouse_vy_win[index] / (float)REMOTE_SMOOTH_TIMES;

  info->mouse_win_idx = index + 1;
}

static int16_t abs_int16(int16_t x) { return x < 0 ? -x : x; }

void rc_sensor_check(rc_sensor_t* sensor) {
  rc_sensor_info_t* info = sensor->info;

  if ((abs_int16(info->thumbwheel.value_last) <
       abs_int16(info->thumbwheel.value)) &&
      (abs_int16(info->thumbwheel_record) < abs_int16(info->thumbwheel.value))) {
    info->thumbwheel_record = info->thumbwheel.value;
  }

  if ((info->thumbwheel.value == 0) && (info->thumbwheel_record != 0)) {
    for (int i = 0; i < 4; i++) {
      if (info->tw_step_value[i] > 0 && info->thumbwheel_record > 0) {
        if (info->thumbwheel_record >= info->tw_step_value[i]) {
          info->thumbwheel.step[i] = !info->thumbwheel.step[i];
          LOG_DBG("Thumbwheel step[%d] toggled to %d", i,
                  info->thumbwheel.step[i]);
          info->thumbwheel_record = 0;
        }
      }
      if (info->tw_step_value[i] < 0 && info->thumbwheel_record < 0) {
        if (info->thumbwheel_record <= info->tw_step_value[i]) {
          info->thumbwheel.step[i] = !info->thumbwheel.step[i];
          LOG_DBG("Thumbwheel step[%d] toggled to %d", i,
                  info->thumbwheel.step[i]);
          info->thumbwheel_record = 0;
        }
      }
    }
    info->thumbwheel_record = 0;
  }

  info->thumbwheel.value_last = info->thumbwheel.value;

  if (abs_int16(info->ch0) > 660 || abs_int16(info->ch1) > 660 ||
      abs_int16(info->ch2) > 660 || abs_int16(info->ch3) > 660) {
    sensor->err = DEV_DATA_ERR;
    LOG_WRN("Remote Data Err: ch0=%d ch1=%d ch2=%d ch3=%d", info->ch0, info->ch1,
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

void rc_sensor_heart_beat(rc_sensor_t* sensor) {
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

void rc_reset_data(rc_sensor_t* sensor) {
  sensor->info->ch0 = 0;
  sensor->info->ch1 = 0;
  sensor->info->ch2 = 0;
  sensor->info->ch3 = 0;
  sensor->info->s1 = RC_SW_MID;
  sensor->info->s2 = RC_SW_MID;
  sensor->info->mode_switch = 0;
  sensor->info->mouse_vx = 0;
  sensor->info->mouse_vy = 0;
  sensor->info->mouse_vz = 0;
  sensor->info->mouse_x = 0.f;
  sensor->info->mouse_y = 0.f;
  sensor->info->mouse_z = 0.f;
  sensor->info->mouse_btn_l.value = 0;
  sensor->info->mouse_btn_r.value = 0;
  sensor->info->mouse_btn_m.value = 0;
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
  sensor->info->thumbwheel_record = 0;

  sensor->info->stop = 0;
  sensor->info->left_button = 0;
  sensor->info->right_button = 0;
  sensor->info->shutter = 0;

  for (int i = 0; i < REMOTE_SMOOTH_TIMES; i++) {
    sensor->info->mouse_vx_win[i] = 0;
    sensor->info->mouse_vy_win[i] = 0;
  }
  sensor->info->mouse_win_idx = 0;

  sensor->info->tt1 = 0;
  sensor->info->tt2 = 0;
  sensor->info->ttp = 0;

  /* offline_cnt 故意不动：初始化时被置为 offline_max_cnt + 1，
   * 用来保证上电后先处于掉线态，等收到第一帧再转在线。
   */
}
