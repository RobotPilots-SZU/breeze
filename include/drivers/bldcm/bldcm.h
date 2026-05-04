#include <zephyr/device.h>
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct smotor_heartbeat_status_t
    {
        uint64_t heartbeat_tick; // 心跳时间戳
        // uint64_t probe_tick;     // 心跳探测时间戳
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

    typedef struct smotor_lk_rxdata_t
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

    typedef struct lk_special_api lk_special_api_t;
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
            const lk_special_api_t *lk_api;
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


#ifdef __cplusplus
}
#endif
