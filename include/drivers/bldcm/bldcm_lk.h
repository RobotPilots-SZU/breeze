#ifndef __BREEZE_DRIVERS_BLDCM_LK_H__
#define __BREEZE_DRIVERS_BLDCM_LK_H__

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include "bldcm.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct motor_lk_control_param {
	uint8_t controlParamID;
	int16_t anglePidKp;
	int16_t anglePidKi;
	int16_t anglePidKd;
	int16_t speedPidKp;
	int16_t speedPidKi;
	int16_t speedPidKd;
	int16_t currentPidKp;
	int16_t currentPidKi;
	int16_t currentPidKd;
	int16_t torqueLimit;
	int32_t speedLimit;
	int32_t angleLimit;
	int32_t currentRamp;
	int32_t speedRamp;
} motor_lk_control_param_t;

typedef struct motor_lk_encoder {
	uint16_t encoder;
	uint16_t encoderRaw;
	uint16_t encoderOffset;
} motor_lk_encoder_t;

typedef struct motor_lk_single_data {
	motor_lk_control_param_t control_param;
	motor_lk_encoder_t encoder_data;
    int64_t motorAngle;                 // 累计电机角度，单位0.01°/LSB
    uint32_t circleAngle;               // 单圈电机角度值，数值范围0~36000*减速比-1，单位0.01°/LSB
} motor_lk_single_data_t;

typedef const motor_lk_single_data_t *(*motor_lk_get_single_data)(const struct device *dev);

typedef int (*motor_lk_wparam_anglepid)(const struct device *dev, int16_t angleKp, int16_t angleKi, int16_t angleKd);

typedef int (*motor_lk_wparam_speedpid)(const struct device *dev, int16_t speedKp, int16_t speedKi, int16_t speedKd);

typedef int (*motor_lk_wparam_currentpid)(const struct device *dev, int16_t currentKp, int16_t currentKi, int16_t currentKd);

typedef int (*motor_lk_wparam_torquelimit)(const struct device *dev, int16_t torqueLimit);

typedef int (*motor_lk_wparam_speedlimit)(const struct device *dev, int32_t speedLimit);

typedef int (*motor_lk_wparam_anglelimit)(const struct device *dev, int32_t angleLimit);

typedef int (*motor_lk_wparam_currentramp)(const struct device *dev, int32_t currentRamp);

typedef int (*motor_lk_wparam_speedramp)(const struct device *dev, int32_t speedRamp);

/**
 * @brief 开环控制，仅在MS上可用
 *
 * @param powerControl
 * @param frame
 * @param cfg
 * @return int
 */
typedef int (*motor_lk_single_openloop_control)(const struct device *dev, int16_t powerControl);


/**
 * @brief 单电机的转矩闭环控制命令，仅在MF，MH，MG上可用，
 *        对应的MF电机实际转矩电流是-16.5A~16.5A，MG电机实际转矩电流是-33A~33A
 *
 * @param iqcontrol
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_closedloop_control)(const struct device *dev, int16_t iqcontrol);

/**
 * @brief 速度闭环控制，带有力矩限制，speedControl受上位机限制，反馈和读取电机状态2一致(仅命令字节data[0]不同)
 *
 * @param speedControl
 * @param iqcontrol
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_speedctrl)(const struct device *dev, double TargetspeedControl, int16_t iqcontrol);

/**
 * @brief 多圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°
 *        受上位机的速度，位置等最大值限制。反馈和读取电机状态2一致(仅命令字节data[0]不同)
 * @param angleControl
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_mulposctrl1)(const struct device *dev, double TargetAngle);

/**
 * @brief 多圈位置闭环控制命令2，带有速度限制，对应实际位置为0.01degree/LSB,也就是36000代表360°，
 *        最大转速对应实际转速1dps/LSB,也就是360代表360dps
 *        受上位机的速度，位置等最大值限制。反馈和读取电机状态2一致(仅命令字节data[0]不同)
 *
 * @param angleControl
 * @param maxSpeed
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_mulposctrl2)(const struct device *dev, double TargetAngle, double TargetmaxSpeed);

/*
 * @brief 单圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数。
 *
 * @param spindir
 * @param angleControl
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_sigposctrl1)(const struct device *dev, bool spindir, double TargetAngle);

/**
 * @brief 单圈位置闭环控制命令2，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数和速度限制。
 *
 * @param spindir
 * @param angleControl
 * @param maxSpeed
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_sigposctrl2)(const struct device *dev, bool spindir, double TargetAngle, double TargetmaxSpeed);

/**
 * @brief 增量位置闭环控制命令1，对应实际增量位置为0.01degree/LSB,也就是36000代表360°
 *
 * @param angleIncre
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_increposctrl1)(const struct device *dev, double TargetAngleIncre);

/**
 * @brief 增量位置闭环控制命令2，对应实际增量位置为0.01degree/LSB,也就是36000代表360°，带有速度限制
 *
 * @param angleIncre
 * @param maxSpeed
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_increposctrl2)(const struct device *dev, double TargetAngleIncre, double TargetmaxSpeed);

/**
 * @brief 多电机的速度控制命令，speedValue的范围是-32768 ~ 32767dps，分辨率是1dps/LSB
 * 
 * @param dev 
 * @param speedValue 
 * @return int 
 */
typedef int (*motor_lk_multi_speedctrl)(const struct device *dev, double TargetspeedValue);

/**
 * @brief 多电机的位置控制命令，angleValue的范围是-32768 ~ 32767dps，分辨率是0.01degree/LSB，也就是36000代表360°，
 * 
 * @param dev 
 * @param angleValue 
 * @return int 
 */
typedef int (*motor_lk_multi_positctrl)(const struct device *dev, double TargetangleValue);

/**
 * @brief 多电机下的混合命令，motor_cmd就是单电机下的控制命令
          支持0x9A,0x9B,0x9C,0x80,0x88,0x81。其他暂不支持
 * 
 * @param dev 
 * @param motor_cmd 
 * @return int 
 */
typedef int (*motor_lk_multi_mixctrl)(const struct device *dev, uint16_t motor_cmd);

struct lk_special_api
{
    motor_lk_get_single_data get_single_data;
    motor_lk_wparam_anglepid writeparam_anglepid;
    motor_lk_wparam_speedpid writeparam_speedpid;
    motor_lk_wparam_currentpid writeparam_currentpid;
    motor_lk_wparam_torquelimit writeparam_torquelimit;
    motor_lk_wparam_speedlimit writeparam_speedlimit;
    motor_lk_wparam_anglelimit writeparam_anglelimit;
    motor_lk_wparam_currentramp writeparam_currentramp;
    motor_lk_wparam_speedramp writeparam_speedramp;
    motor_lk_single_openloop_control single_openloop_control;
    motor_lk_single_closedloop_control single_closedloop_control;
    motor_lk_single_speedctrl single_speedcontrol;
    motor_lk_single_mulposctrl1 single_mulposctrl1;
    motor_lk_single_mulposctrl2 single_mulposctrl2;
    motor_lk_single_sigposctrl1 single_sigposctrl1;
    motor_lk_single_sigposctrl2 single_sigposctrl2;
    motor_lk_single_increposctrl1 single_increposctrl1;
    motor_lk_single_increposctrl2 single_increposctrl2;
    motor_lk_multi_speedctrl multi_speedcontrol;
    motor_lk_multi_positctrl multi_positcontrol;
    motor_lk_multi_mixctrl multi_mixcontrol;
};


/*-----------------------------------------------------------------------lk special API------------------------------------------------*/    
    /**
     * @brief 获取瓴控电机单电机模式下的特有数据
     * 
     * @param dev 
     * @return const motor_lk_single_data_t* 
     */
    static inline const motor_lk_single_data_t *get_single_data(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->get_single_data == NULL) {
            return NULL;
        }
        return api->lk_api->get_single_data(dev);
    }

    static inline int writeparam_anglepid(const struct device *dev, int16_t angleKp, int16_t angleKi, int16_t angleKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_anglepid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_anglepid(dev, angleKp, angleKi, angleKd);
    }

    static inline int writeparam_speedpid(const struct device *dev, int16_t speedKp, int16_t speedKi, int16_t speedKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_speedpid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_speedpid(dev, speedKp, speedKi, speedKd);
    }

    static inline int writeparam_currentpid(const struct device *dev, int16_t currentKp, int16_t currentKi, int16_t currentKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_currentpid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_currentpid(dev, currentKp, currentKi, currentKd);
    }

    static inline int writeparam_torquelimit(const struct device *dev, int16_t torqueLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_torquelimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_torquelimit(dev, torqueLimit);
    }

    static inline int writeparam_speedlimit(const struct device *dev, int32_t speedLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_speedlimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_speedlimit(dev, speedLimit);
    }

    static inline int writeparam_anglelimit(const struct device *dev, int32_t angleLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_anglelimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_anglelimit(dev, angleLimit);
    }

    static inline int writeparam_currentramp(const struct device *dev, int32_t currentRamp)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_currentramp == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_currentramp(dev, currentRamp);
    }


    static inline int writeparam_speedramp(const struct device *dev, int32_t speedRamp)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->writeparam_speedramp == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->writeparam_speedramp(dev, speedRamp);
    }

    static inline int single_openloop_control(const struct device *dev, int16_t powerControl)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_openloop_control == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_openloop_control(dev, powerControl);
    }

    static inline int single_closedloop_control(const struct device *dev, int16_t iqcontrol)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_closedloop_control == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_closedloop_control(dev, iqcontrol);
    }

    static inline int single_speedcontrol(const struct device *dev, double TargetspeedControl, int16_t iqcontrol)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_speedcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_speedcontrol(dev, TargetspeedControl, iqcontrol);
    }

    static inline int single_mulposctrl1(const struct device *dev, double Targetangle)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_mulposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_mulposctrl1(dev, Targetangle);
    }

    static inline int single_mulposctrl2(const struct device *dev, double Targetangle, double TargetmaxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_mulposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_mulposctrl2(dev, Targetangle, TargetmaxSpeed);
    }

    static inline int single_sigposctrl1(const struct device *dev, bool direction, double TargetAngle)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_sigposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_sigposctrl1(dev, direction, TargetAngle);
    }

    static inline int single_sigposctrl2(const struct device *dev, bool spindir, double Targetangle, double TargetmaxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_sigposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_sigposctrl2(dev, spindir, Targetangle, TargetmaxSpeed);
    }

    static inline int single_increposctrl1(const struct device *dev, double TargetangleIncre)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_increposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_increposctrl1(dev, TargetangleIncre);
    }

    static inline int single_increposctrl2(const struct device *dev,double TargetangleIncre, double TargetmaxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->single_increposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->single_increposctrl2(dev, TargetangleIncre, TargetmaxSpeed);
    }

    static inline int multi_speedcontrol(const struct device *dev, int16_t speedValue)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->multi_speedcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->multi_speedcontrol(dev, speedValue);
    }

    static inline int multi_positcontrol(const struct device *dev, int32_t angleValue)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->multi_positcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->multi_positcontrol(dev, angleValue);
    }

    static inline int multi_mixcontrol(const struct device *dev, uint16_t cmd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
            if(!api || api->lk_api == NULL || api->lk_api->multi_mixcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api->multi_mixcontrol(dev, cmd);
    }
#ifdef __cplusplus
}
#endif

#endif /* __BREEZE_DRIVERS_BLDCM_LK_H__ */