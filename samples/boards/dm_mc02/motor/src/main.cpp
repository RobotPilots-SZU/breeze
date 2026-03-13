#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/motor.h>
#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif
#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define MOTOR_3508_CURRENT_MAX 10000
#define MOTOR_2006_CURRENT_MAX 8000

#define CHASSIS_FL_NODE DT_NODELABEL(chassis_fl)
#define CHASSIS_FR_NODE DT_NODELABEL(chassis_fr)
#ifdef CONFIG_CAN_RX_MANAGER
#define RX_MANAGER_NODE DT_NODELABEL(can_rx_mgr1)
#endif

static float angle_rad = 0.0f;        // 正弦函数的角度（弧度）
static const float ANGLE_STEP = 0.1f; // 每次调用的角度步长（控制正弦频率）
#define M_PI 3.14159265358979323846   /* pi */

int main(void)
{
    LOG_INF("[app] start");
    const struct device *motor_fl = DEVICE_DT_GET(CHASSIS_FL_NODE);
    const struct device *motor_fr = DEVICE_DT_GET(CHASSIS_FR_NODE);
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr   = DEVICE_DT_GET(RX_MANAGER_NODE);
#endif
    const char *motor_type = DT_PROP(CHASSIS_FL_NODE, motor_type);
    if (!motor_fl) {
        LOG_ERR("motor FL not found");
        return -ENODEV;
    }
    if (!motor_fr) {
        LOG_ERR("motor FR not found");
        return -ENODEV;
    }

    if (!device_is_ready(motor_fl)) {
        LOG_ERR("motor FL not ready: %s", motor_fl->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_fr)) {
        LOG_ERR("motor FR not ready: %s", motor_fr->name);
        return -ENODEV;
    }
#ifdef CONFIG_CAN_RX_MANAGER
    if(!rx_mgr) {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if(!device_is_ready(rx_mgr)) {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr->name);
        return -ENODEV;
    }
#endif
    int current_max = 0;

    if(strcmp(motor_type, "M3508") == 0)
    {
        current_max = MOTOR_3508_CURRENT_MAX;
    }
    else if(strcmp(motor_type, "M2006") == 0)
    {
        current_max = MOTOR_2006_CURRENT_MAX;
    }
    else
    {
        LOG_WRN("Unknown motor type '%s', using default current max", motor_type);
        current_max = 1000;
    }


    register_motor(motor_fl);
    register_motor(motor_fr);

    while (true) {
        float sin_val = sin(angle_rad);
        float current_float = current_max * 0.15f * sin_val;
        int current = (int)round(current_float);

        // 更新角度（循环0~2π，实现正弦波循环）
        angle_rad += ANGLE_STEP;
        if (angle_rad >= 2 * M_PI)
        {
            angle_rad = 0.0f;
        }

        const smotor_receive_data_t *fl = get_motor_rxdata(motor_fl);
        const smotor_receive_data_t *fr = get_motor_rxdata(motor_fr);

        // 接收函数检查
        LOG_INF("FL angle=%d speed=%d current=%d alive=%d temp=%d",
                    (int)fl->angle, (int)fl->speed, (int)fl->current,
                    get_motor_heartbeat_status(motor_fl) ? 1 : 0, (int)fl->specific_data.m3508.temp);
        LOG_INF("FR angle=%d speed=%d current=%d alive=%d temp=%d",
                (int)fr->angle, (int)fr->speed, (int)fr->current,
                get_motor_heartbeat_status(motor_fr) ? 1 : 0, (int)fr->specific_data.m3508.temp);

        // 发送函数检查
        motor_update_serialized(motor_fl, current);
        motor_update_serialized(motor_fr, current);
        // can_tx_manager_send(can_tx_mgr, K_FOREVER, NULL,
        //                     0x200, NULL);
        k_sleep(K_MSEC(100));
    }

    return 0;
}
