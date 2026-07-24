#ifndef __BREEZE_DRIVERS_BLDCM_DM_H__
#define __BREEZE_DRIVERS_BLDCM_DM_H__

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include "bldcm.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
    DM_MODE_MIT = 0,      // MIT模式
    DM_MODE_POS_VEL,      // 位置速度模式
    DM_MODE_VEL,          // 速度模式
} dm_control_mode_t;

typedef struct motor_dm_config_param {
    float pMax;             // 位置最大值，单位rad
    float vMax;             // 速度最大值，单位rad/s
    float tMax;             // 扭矩最大值，单位nm
    dm_control_mode_t mode; // 控制模式
} motor_dm_config_param_t;

/**
 * @brief 达妙电机寄存器读接口
 * @param dev 电机设备指针
 * @param reg_addr 寄存器地址
 *
 */
typedef int (*motor_dm_read_reg)(const struct device *dev, uint8_t reg_addr);

/**
 * @brief 达妙电机写寄存器值接口
 * @param dev 电机设备指针
 * @param reg_addr 寄存器地址
 * @param data 寄存器数据
 *
 */
typedef int (*motor_dm_write_reg)(const struct device *dev, uint8_t reg_addr, uint32_t data);

/**
 * @brief 达妙电机保存寄存器值接口
 * @param dev 电机设备指针
 * @param reg_addr 寄存器地址
 *
 */
typedef int (*motor_dm_store_reg)(const struct device *dev, uint8_t reg_addr);

/**
 * @brief 达妙电机设置当前位置为零点
 *
 */
typedef int (*motor_dm_save_zero)(const struct device *dev);

/**
 * @brief 达妙电机MIT控制模式
 * @param dev 电机设备
 * @param pos 位置值，范围由上位机设定
 * @param vel 速度值，范围由上位机设定
 * @param Kp 位置环比例增益，范围由上位机设定
 * @param Kd 位置环微分增益，范围由上位机设定
 * @param Tq 扭矩值，范围由上位机设定
 *
 */
typedef int (*motor_dm_mit_control)(const struct device *dev, float pos, float vel, float Kp, float Kd, float Tq);

/**
 * @brief 达妙电机位置速度控制模式
 * @param dev 电机设备
 * @param pos 位置值
 * @param vel 这个速度是梯形加速度运行下的最高速度，也就是匀速段的速度值。
 *
 */
typedef int (*motor_dm_posvel_control)(const struct device *dev, float pos, float vel);

/**
 * @brief 达妙电机速度控制模式，需要注意这个报文的dlc应该是4
 * @param dev 电机设备
 * @param vel 速度值
 *
 */
typedef int (*motor_dm_vel_control)(const struct device *dev, float vel);

struct dm_special_api
{
    motor_dm_read_reg dm_read_reg;
    motor_dm_write_reg dm_write_reg;
    motor_dm_store_reg dm_store_reg;
    motor_dm_save_zero dm_save_zero;
    motor_dm_mit_control dm_mit_control;
    motor_dm_posvel_control dm_posvel_control;
    motor_dm_vel_control dm_vel_control;
};


/*-----------------------------------------------------------------------dm special API------------------------------------------------*/
    /**
     * @brief 读取达妙电机寄存器
     *
     * @param dev
     * @param reg_addr 寄存器地址
     * @return int
     */
    static inline int dm_read_reg(const struct device *dev, uint8_t reg_addr)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_read_reg == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_read_reg(dev, reg_addr);
    }

    static inline int dm_write_reg(const struct device *dev, uint8_t reg_addr, uint32_t data)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_write_reg == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_write_reg(dev, reg_addr, data);
    }

    static inline int dm_store_reg(const struct device *dev, uint8_t reg_addr)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_store_reg == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_store_reg(dev, reg_addr);
    }

    static inline int dm_save_zero(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_save_zero == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_save_zero(dev);
    }

    static inline int dm_mit_control(const struct device *dev, float pos, float vel, float Kp, float Kd, float Tq)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_mit_control == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_mit_control(dev, pos, vel, Kp, Kd, Tq);
    }

    static inline int dm_posvel_control(const struct device *dev, float pos, float vel)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_posvel_control == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_posvel_control(dev, pos, vel);
    }

    static inline int dm_vel_control(const struct device *dev, float vel)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->dm_api == NULL || api->dm_api->dm_vel_control == NULL) {
            return -ENOSYS;
        }
        return api->dm_api->dm_vel_control(dev, vel);
    }
#ifdef __cplusplus
}
#endif

#endif /* __BREEZE_DRIVERS_BLDCM_DM_H__ */