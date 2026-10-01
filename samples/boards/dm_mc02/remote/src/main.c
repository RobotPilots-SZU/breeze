#include "rc_thread.h"

#include <drivers/remote/rc_common.h>
#include <drivers/remote/remote.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(remote_app, LOG_LEVEL_INF);

#define DR16_REMOTE_NODE DT_ALIAS(remote0)
#if !DT_NODE_HAS_STATUS_OKAY(DR16_REMOTE_NODE)
#error "DT alias 'remote0' 未定义或禁用"
#endif

static const struct device* remote_dev;
static rc_sensor_t* sensor;

/**
 *	@brief	按键状态机的 1ms 节拍
 *
 * rc_keyboard_update() 每调一次给按住时长 +1，所以固定 1ms 调用时
 * key_board_info_t::cnt 的单位就是毫秒，与 KEY_*_CNT_MAX 的注释对应。
 *
 * 回调运行在系统时钟中断上下文。掉线时直接跳过：否则 value 会停在最后一帧的
 * 按下态，cnt 一路涨到 cnt_max，按键被永久判成长按。
 */
static void rc_key_tick(struct k_timer* timer) {
  ARG_UNUSED(timer);

  if (sensor != NULL && sensor->is_online) {
    rc_keyboard_update(sensor->info);
  }
}

K_TIMER_DEFINE(rc_key_timer, rc_key_tick, NULL);

struct app_data {
  uint32_t last_receive_time;
  uint32_t packet_count;
};

static struct app_data my_app_data = {
    .last_receive_time = 0,
    .packet_count = 0,
};

static void remote_data_ready(const struct device* dev, rc_sensor_t* sns, void* user_data) {
  struct app_data* data = (struct app_data*)user_data;
  data->last_receive_time = k_uptime_get_32();
  data->packet_count++;
}

int remote_app_init(void) {
  remote_dev = DEVICE_DT_GET(DR16_REMOTE_NODE);
  if (!device_is_ready(remote_dev)) {
    LOG_ERR("Remote device not ready");
    return -1;
  }
  sensor = remote_get_sensor(remote_dev);
  if (!sensor) {
    LOG_ERR("Failed to get remote sensor");
    return -1;
  }

  /* 组帧 + 解析线程在应用侧，见 src/rc_thread.c */
  if (rc_parse_attach(remote_dev) < 0) {
    LOG_ERR("Failed to attach remote parser");
    return -1;
  }
  rc_parse_set_data_ready_cb(remote_dev, remote_data_ready, &my_app_data);

  /* 1ms 周期推进按键状态机，见 rc_key_tick() */
  k_timer_start(&rc_key_timer, K_MSEC(1), K_MSEC(1));
  return 0;
}

int main(void) {
  if (remote_app_init() < 0) {
    return -1;
  }
  while (1) {
    if (!sensor->is_online) {
      printk("\033[2J\033[H");
      printk("====================================\n");
      printk("         DJI DR16 RECEIVER          \n");
      printk("====================================\n");
      printk(" STATUS: OFFLINE\n");
    } else {
      printk("\033[2J\033[H");
      printk("==============================================\n");
      printk("              DJI DR16 RECEIVER               \n");
      printk("==============================================\n");
      printk(" STATUS: ONLINE   |  PACKETS: %-6u\n", my_app_data.packet_count);
      printk(" UPTIME: %-8u |  S1: %d  S2: %d\n", my_app_data.last_receive_time, sensor->info->s1, sensor->info->s2);
      printk("----------------------------------------------\n");
      printk(" [ RIGHT STICK ]  |  [ LEFT STICK ] \n");
      printk("  CH0 (X): %-6d |   CH2 (X): %-6d\n", sensor->info->ch0, sensor->info->ch2);
      printk("  CH1 (Y): %-6d |   CH3 (Y): %-6d\n", sensor->info->ch1, sensor->info->ch3);
      printk("----------------------------------------------\n");
      printk(" [ THUMBWHEEL ]   :  %-6d  (Steps: %d%d%d%d)\n",
             sensor->info->thumbwheel.value,
             sensor->info->thumbwheel.step[0], sensor->info->thumbwheel.step[1],
             sensor->info->thumbwheel.step[2], sensor->info->thumbwheel.step[3]);
      printk("----------------------------------------------\n");
      printk(" [ MOUSE AXES ]   |  [ MOUSE BUTTONS ]\n");
      printk("  VX: %-6d      |   LEFT:  %d (Cnt: %d)\n", sensor->info->mouse_vx, sensor->info->mouse_btn_l.value, sensor->info->mouse_btn_l.cnt);
      printk("  VY: %-6d      |   RIGHT: %d (Cnt: %d)\n", sensor->info->mouse_vy, sensor->info->mouse_btn_r.value, sensor->info->mouse_btn_r.cnt);
      printk("  VZ: %-6d      |\n", sensor->info->mouse_vz);
      printk("----------------------------------------------\n");
      printk(" [ KEYBOARD MAP ] Raw Vector: 0x%04X\n", sensor->info->key_v);
      printk("  W:%d S:%d A:%d D:%d | Q:%d E:%d R:%d F:%d | G:%d Z:%d X:%d C:%d\n",
             sensor->info->W.value, sensor->info->S.value, sensor->info->A.value, sensor->info->D.value,
             sensor->info->Q.value, sensor->info->E.value, sensor->info->R.value, sensor->info->F.value,
             sensor->info->G.value, sensor->info->Z.value, sensor->info->X.value, sensor->info->C.value);
      printk("  V:%d B:%d        | SHIFT:%d CTRL:%d\n",
             sensor->info->V.value, sensor->info->B.value, sensor->info->Shift.value, sensor->info->Ctrl.value);
      printk("==============================================\n");
    }
    // Refresh the screen at 10 Hz
    k_sleep(K_MSEC(100));
  }
  return 0;
}