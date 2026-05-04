#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
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
typedef int (*motor_lk_single_speedctrl)(const struct device *dev, int32_t speedControl, int16_t iqcontrol);

/**
 * @brief 多圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°
 *        受上位机的速度，位置等最大值限制。反馈和读取电机状态2一致(仅命令字节data[0]不同)
 * @param angleControl
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_mulposctrl1)(const struct device *dev, int32_t angleControl);

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
typedef int (*motor_lk_single_mulposctrl2)(const struct device *dev, int32_t angleControl, uint16_t maxSpeed);

/*
 * @brief 单圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数。
 *
 * @param spindir
 * @param angleControl
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_sigposctrl1)(const struct device *dev, bool spindir, uint32_t angleControl);

/**
 * @brief 单圈位置闭环控制命令2，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数和速度限制。
 *
 * @param spindir
 * @param angleControl
 * @param maxSpeed
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_sigposctrl2)(const struct device *dev, bool spindir, uint32_t angleControl, uint16_t maxSpeed);

/**
 * @brief 增量位置闭环控制命令1，对应实际增量位置为0.01degree/LSB,也就是36000代表360°
 *
 * @param angleIncre
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_increposctrl1)(const struct device *dev, int32_t angleIncre);

/**
 * @brief 增量位置闭环控制命令2，对应实际增量位置为0.01degree/LSB,也就是36000代表360°，带有速度限制
 *
 * @param angleIncre
 * @param maxSpeed
 * @param frame
 * @return int
 */
typedef int (*motor_lk_single_increposctrl2)(const struct device *dev, int32_t angleIncre, uint16_t maxSpeed);

/**
 * @brief 多电机的速度控制命令，speedValue的范围是-32768 ~ 32767dps，分辨率是1dps/LSB
 * 
 * @param dev 
 * @param speedValue 
 * @return int 
 */
typedef int (*motor_lk_multi_speedctrl)(const struct device *dev, int16_t speedValue);

/**
 * @brief 多电机的位置控制命令，angleValue的范围是-32768 ~ 32767dps，分辨率是0.01degree/LSB，也就是36000代表360°，
 * 
 * @param dev 
 * @param angleValue 
 * @return int 
 */
typedef int (*motor_lk_multi_positctrl)(const struct device *dev, int32_t angleValue);

/**
 * @brief 多电机下的混合命令，motor_cmd就是单电机下的控制命令
          支持0x9A,0x9B,0x9C,0x80,0x88,0x81。其他暂不支持
 * 
 * @param dev 
 * @param motor_cmd 
 * @return int 
 */
typedef int (*motor_lk_multi_mixctrl)(const struct device *dev, uint16_t motor_cmd);


typedef struct lk_special_api
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
}lk_special_api_t;
