#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/bldcm/bldcm.h>
#include <drivers/bldcm/bldcm_dm.h>
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
#define MOTOR_MG_CURRENT_MAX 2000

#define CHASSIS_FL_NODE DT_NODELABEL(chassis_fl)
#define CHASSIS_FR_NODE DT_NODELABEL(chassis_fr)
#define ARM_JOINT1_NODE DT_NODELABEL(arm_joint1)        // lk电机
#define ARM_JOINT2_NODE DT_NODELABEL(arm_joint2)        // dm电机

static float angle_rad = 0.0f;        // 正弦函数的角度（弧度）
static const float ANGLE_STEP = 0.1f; // 每次调用的角度步长（控制正弦频率）
#define M_PI 3.14159265358979323846   /* pi */

float choose_current_max(char *motor_type) {
    if(motor_type == NULL)
    {
        return MOTOR_3508_CURRENT_MAX; // 默认值
    }
    if(strcmp(motor_type, "M3508") == 0) {
        return MOTOR_3508_CURRENT_MAX;
    } else if(strcmp(motor_type, "M2006") == 0) {
        return MOTOR_2006_CURRENT_MAX;
    } else if(strcmp(motor_type, "DM80") == 0) {
        return DT_PROP(ARM_JOINT2_NODE, tq_max) / 1000.0f;          // 达妙电机的最大扭矩电流是上位机确定的，因此需要从设备树获取
    } else if (strcmp(motor_type, "MG") == 0) {
        return MOTOR_MG_CURRENT_MAX;
    } else {
        return 0;           // 未知电机不转
    }
}


int main(void)
{
    LOG_INF("[app] start");
    const struct device *motor_fl = DEVICE_DT_GET(CHASSIS_FL_NODE);
    const struct device *motor_fr = DEVICE_DT_GET(CHASSIS_FR_NODE);
    const struct device *arm_joint1 = DEVICE_DT_GET(ARM_JOINT1_NODE);
    const struct device *arm_joint2 = DEVICE_DT_GET(ARM_JOINT2_NODE);

    if (!motor_fl) {
        LOG_ERR("motor FL not found");
        return -ENODEV;
    }
    if (!motor_fr) {
        LOG_ERR("motor FR not found");
        return -ENODEV;
    }
    if (!arm_joint1) {
        LOG_ERR("arm_joint1 not found");
        return -ENODEV;
    }    
    if (!arm_joint2) {
        LOG_ERR("arm_joint2 not found");
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
    if (!device_is_ready(arm_joint1)) {
        LOG_ERR("arm_joint1 not ready: %s", arm_joint1->name);
        return -ENODEV;
    }
    if (!device_is_ready(arm_joint2)) {
        LOG_ERR("joint_dm not ready: %s", arm_joint2->name);
        return -ENODEV;
    }

    float fl_current_max = choose_current_max((char *)DT_PROP(CHASSIS_FL_NODE, motor_type));
    float fr_current_max = choose_current_max((char *)DT_PROP(CHASSIS_FR_NODE, motor_type));
    float lk_current_max = choose_current_max((char *)DT_PROP(ARM_JOINT1_NODE, motor_type));
    float dm_current_max = choose_current_max((char *)DT_PROP(ARM_JOINT2_NODE, motor_type));

    motor_enable(arm_joint2);     // 达妙电机必须先使能

    register_motor(motor_fl);
    register_motor(motor_fr);
    register_motor(arm_joint1);
    register_motor(arm_joint2);

    while (true) {
        float sin_val = sin(angle_rad);
        float flcurrent_float = fl_current_max * 0.15f * sin_val;
        float frcurrent_float = fr_current_max * 0.15f * sin_val;
        float lkcurrent_flaot = lk_current_max * 0.15f * sin_val;
        float dmcurrent_float = dm_current_max * 0.15f * sin_val;
        int flcurrent = (int)round(flcurrent_float);
        int frcurrent = (int)round(frcurrent_float);
        int lkcurrent = (int)round(lkcurrent_flaot);
        int dmcurrent = (int)round(dmcurrent_float);

        // 更新角度（循环0~2π，实现正弦波循环）
        angle_rad += ANGLE_STEP;
        if ((double)angle_rad >= 2 * M_PI)
        {
            angle_rad = 0.0f;
        }

        const smotor_receive_data_t *fl = get_motor_rxdata(motor_fl);
        const smotor_receive_data_t *fr = get_motor_rxdata(motor_fr);
        const smotor_receive_data_t *joint1 = get_motor_rxdata(arm_joint1);
        const smotor_receive_data_t *joint2 = get_motor_rxdata(arm_joint2);

        if(get_motor_heartbeat_status(motor_fl) && fl != NULL) {
            LOG_INF("FL raw_angle=%d speed=%d current=%d alive=%d temp=%d",
                        (int)fl->encoder, (int)fl->speed, (int)fl->iq,
                        get_motor_heartbeat_status(motor_fl) ? 1 : 0, (int)fl->specific_data.m3508.temp);
        }
        if(get_motor_heartbeat_status(motor_fr) && fr != NULL) {
            LOG_INF("FR raw_angle=%d speed=%d current=%d alive=%d temp=%d",
                        (int)fr->encoder, (int)fr->speed, (int)fr->iq,
                        get_motor_heartbeat_status(motor_fr) ? 1 : 0, (int)fr->specific_data.m3508.temp);
        }
        if(get_motor_heartbeat_status(arm_joint1) && joint1 != NULL) {
            LOG_INF("JOINT_LK raw_angle=%d raw_speed=%d current=%d alive=%d",
                        joint1->encoder, (int)joint1->speed, (int)joint1->iq,
                        get_motor_heartbeat_status(arm_joint1) ? 1 : 0);
        }
        if(get_motor_heartbeat_status(arm_joint2) && joint2 != NULL) {
            LOG_INF("JOINT_DM raw_angle=%d raw_speed=%d angle=%.2f speed=%.2f alive=%d",
                    (int)joint2->encoder, (int)joint2->speed, (double)joint2->specific_data.dm.pos_real, (double)joint2->specific_data.dm.vel_real, 
                    get_motor_heartbeat_status(arm_joint2) ? 1 : 0);
        }

        motor_torque_control(motor_fl, flcurrent);
        motor_torque_control(motor_fr, frcurrent);
        motor_torque_control(arm_joint1, lkcurrent);
        dm_mit_control(arm_joint2, 0.0f, 0.0f, 0.0f, 0.0f, dmcurrent);
        
        k_sleep(K_MSEC(100));
    }

    return 0;
}
