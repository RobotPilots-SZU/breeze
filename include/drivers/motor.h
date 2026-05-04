#include <zephyr/device.h>
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "lk_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct smotor_heartbeat_status_t
    {
        uint64_t heartbeat_tick; // 心跳时间戳
        uint64_t probe_tick;     // 心跳探测时间戳
        bool is_alive;           // 心跳状态
    } smotor_heartbeat_status_t;

    /* M2006特有的数据结构，暂时是空*/
    // typedef struct smotor_m2006_rxdata_t
    // {

    // } smotor_m2006_rxdata_t;

    /* M3508特有的数据结构*/
    typedef struct smotor_m3508_rxdata_t
    {
        int16_t temp;               // 电机温度值
    } smotor_m3508_rxdata_t;

    typedef struct smotor_m6020_rxdata_t
    {
        int16_t temp;               // 电机温度值
    } smotor_m6020_rxdata_t;

    typedef struct smotor_kt_rxdata_t
    {
        int8_t temp;                // 电机温度值 1°C/LSB
	    int16_t vol;                // 母线电压值 0.01V/LSB
        int16_t current;            // 母线电流值 0.01A/LSB
        int16_t power;              // 电机功率值，只在MS电机上存在 -1000~1000
        int16_t iA;                 // A相电流值，MS电机上不存在
        int16_t iB;                 // B相电流值，MS电机上不存在
        int16_t iC;                 // C相电流值，MS电机上不存在
        uint8_t motorState;         // 电机状态：0x00-电机处于开启状态, 0x10-电机处于关闭状态
        uint8_t errorState;         // 电机错误状态：0x00-无错误, 0x01-低压, 0x02-过压, 0x04-驱动过温,
                                    // 0x08-电机过温, 0x10-电机过流, 0x20-短路, 0x40-堵转, 0x80-信号丢失
        uint8_t holdBrakeState;     // 抱闸器状态：0x00-抱闸器启动, 0x01-抱闸器释放
    } smotor_lk_rxdata_t;

    typedef struct smotor_receive_data_t
    {
        int16_t speed;              // 速度值
        int32_t encoder;            // 编码器原始值
        int16_t iq;                 // 扭矩电流值
        uint32_t valid_mask;        // 有效数据掩码
        union {
            smotor_m3508_rxdata_t m3508;
            smotor_m6020_rxdata_t m6020;
            smotor_lk_rxdata_t lk;
            // smotor_m2006_rxdata_t m2006;
        } specific_data;         // 不同电机类型的特有数据
    } smotor_receive_data_t;

    typedef enum motor_rx_valid_t
    {
        MOTOR_RX_VALID_IQ       = 1u << 0,
        MOTOR_RX_VALID_SPEED    = 1u << 1,
        MOTOR_RX_VALID_ENCODER  = 1u << 2,
        MOTOR_DJI_3508          = 1u << 3,
        MOTOR_DJI_6020          = 1u << 4,
        MOTOR_LK                = 1u << 5,
    } motor_rx_valid_t;

    /**
     * @brief 掩码判断接收数据字段是否有效
     *
     * @param rx
     * @param mask
     * @return true
     * @return false
     */
    static inline bool motor_rx_has(const smotor_receive_data_t *rx, uint32_t mask)
    {
        return (rx != NULL) && ((rx->valid_mask & mask) == mask);
    }

    typedef struct smotor_data_t
    {
        uint8_t tx_data[8];
        smotor_receive_data_t rx_data;
        smotor_heartbeat_status_t heartbeat_status;
        void *interface_ptr;                            // 指向具体接口的指针
    } smotor_data_t;

    /**
     * @typedef motor_api_register
     * @brief Callback API for register a motor device.
     *
     */
    typedef int (*motor_api_register)(const struct device *dev);


    /**
     * @typedef motor_api_get_rxdata
     * @brief get motor receive data pointer
     *
     */
    typedef const smotor_receive_data_t *(*motor_api_get_rxdata)(const struct device *dev);

    typedef int (*motor_api_torque_control)(const struct device *dev, int16_t current);

    /**
     * @typedef motor_api_get_heartbeat_status
     * @brief application/middleware get motor heartbeat status
     *
     */
    typedef int (*motor_api_get_heartbeat_status)(const struct device *dev);

    typedef int (*motor_api_change_tx_feq)(const struct device *dev, uint16_t new_feq);

    typedef int (*motor_api_clear_error)(const struct device *dev);

    typedef int (*motor_api_disable)(const struct device *dev);

    typedef int (*motor_api_enable)(const struct device *dev);

    typedef int (*motor_api_stop)(const struct device *dev);


    typedef struct motor_driver_api_t
    {
        motor_api_register register_motor;
        motor_api_get_rxdata get_rxdata;
        motor_api_change_tx_feq change_tx_feq;
        motor_api_get_heartbeat_status get_heartbeat_status;
        motor_api_torque_control torque_control;
        motor_api_clear_error clear_error;
        motor_api_disable disable;
        motor_api_enable enable;
        motor_api_stop stop;
        union{
            lk_special_api_t lk_api;
        };
    } motor_driver_api_t;

    static inline int register_motor(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->register_motor == NULL) {
            return -ENOSYS;
        }
        return api->register_motor(dev);
    }

    static inline int get_motor_heartbeat_status(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->get_heartbeat_status == NULL) {
            return -ENOSYS;
        }
        return api->get_heartbeat_status(dev);
    }

    static inline const smotor_receive_data_t *get_motor_rxdata(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->get_rxdata == NULL) {
            return NULL;
        }
        return api->get_rxdata(dev);
    }

    static inline int motor_torque_control(const struct device *dev, int16_t current)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->torque_control == NULL) {
            return -ENOSYS;
        }
        return api->torque_control(dev, current);
    }

    static inline int motor_change_tx_feq(const struct device *dev, uint16_t new_feq)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->change_tx_feq == NULL) {
            return -ENOSYS;
        }
        return api->change_tx_feq(dev, new_feq);
    }

    static inline int motor_clear_error(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->clear_error == NULL) {
            return -ENOSYS;
        }
        return api->clear_error(dev);
    }

    static inline int motor_disable(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->disable == NULL) {
            return -ENOSYS;
        }
        return api->disable(dev);
    }

    static inline int motor_enable(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->enable == NULL) {
            return -ENOSYS;
        }
        return api->enable(dev);
    }

    static inline int motor_stop(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->stop == NULL) {
            return -ENOSYS;
        }
        return api->stop(dev);
    }

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
        if(!api || api->lk_api.get_single_data == NULL) {
            return NULL;
        }
        return api->lk_api.get_single_data(dev);
    }

    static inline int writeparam_anglepid(const struct device *dev, int16_t angleKp, int16_t angleKi, int16_t angleKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_anglepid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_anglepid(dev, angleKp, angleKi, angleKd);
    }

    static inline int writeparam_speedpid(const struct device *dev, int16_t speedKp, int16_t speedKi, int16_t speedKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_speedpid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_speedpid(dev, speedKp, speedKi, speedKd);
    }

    static inline int writeparam_currentpid(const struct device *dev, int16_t currentKp, int16_t currentKi, int16_t currentKd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_currentpid == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_currentpid(dev, currentKp, currentKi, currentKd);
    }

    static inline int writeparam_torquelimit(const struct device *dev, int16_t torqueLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_torquelimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_torquelimit(dev, torqueLimit);
    }

    static inline int writeparam_speedlimit(const struct device *dev, int32_t speedLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_speedlimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_speedlimit(dev, speedLimit);
    }

    static inline int writeparam_anglelimit(const struct device *dev, int32_t angleLimit)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_anglelimit == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_anglelimit(dev, angleLimit);
    }

    static inline int writeparam_currentramp(const struct device *dev, int32_t currentRamp)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_currentramp == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_currentramp(dev, currentRamp);
    }


    static inline int writeparam_speedramp(const struct device *dev, int32_t speedRamp)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.writeparam_speedramp == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.writeparam_speedramp(dev, speedRamp);
    }

    static inline int single_openloop_control(const struct device *dev, int16_t powerControl)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_openloop_control == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_openloop_control(dev, powerControl);
    }

    static inline int single_closedloop_control(const struct device *dev, int16_t iqcontrol)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_closedloop_control == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_closedloop_control(dev, iqcontrol);
    }

    static inline int single_speedcontrol(const struct device *dev, int32_t speedControl, int16_t iqcontrol)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_speedcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_speedcontrol(dev, speedControl, iqcontrol);
    }

    static inline int single_mulposctrl1(const struct device *dev, int32_t angleControl)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_mulposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_mulposctrl1(dev, angleControl);
    }

    static inline int single_mulposctrl2(const struct device *dev, int32_t angleControl, uint16_t maxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_mulposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_mulposctrl2(dev, angleControl, maxSpeed);
    }

    static inline int single_sigposctrl1(const struct device *dev, int32_t angleControl, uint8_t direction)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_sigposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_sigposctrl1(dev, angleControl, direction);
    }

    static inline int single_sigposctrl2(const struct device *dev, int32_t angleControl, uint8_t direction, uint16_t maxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_sigposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_sigposctrl2(dev, angleControl, direction, maxSpeed);
    }

    static inline int single_increposctrl1(const struct device *dev, int32_t angleIncre)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_increposctrl1 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_increposctrl1(dev, angleIncre);
    }

    static inline int single_increposctrl2(const struct device *dev, int32_t angleIncre, uint16_t maxSpeed)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.single_increposctrl2 == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.single_increposctrl2(dev, angleIncre, maxSpeed);
    }

    static inline int multi_speedcontrol(const struct device *dev, int16_t speedValue)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.multi_speedcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.multi_speedcontrol(dev, speedValue);
    }

    static inline int multi_positcontrol(const struct device *dev, int32_t angleValue)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.multi_positcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.multi_positcontrol(dev, angleValue);
    }

    static inline int multi_mixcontrol(const struct device *dev, uint16_t cmd)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->lk_api.multi_mixcontrol == NULL) {
            return -ENOSYS;
        }
        return api->lk_api.multi_mixcontrol(dev, cmd);
    }
#ifdef __cplusplus
}
#endif
