#include <zephyr/device.h>
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct sMotor_Heartbeat_Status_t
    {
        uint64_t heartbeat_tick; // 心跳时间戳
        bool is_alive;           // 心跳状态
    } sMotor_Heartbeat_Status_t;

    /* M2006特有的数据结构，暂时是空*/
    // typedef struct sMotor_M2006_RxData_t
    // {

    // } sMotor_M2006_RxData_t;

    /* M3508特有的数据结构*/
    typedef struct sMotor_M3508_RxData_t
    {
        int16_t temp;               // 电机温度值
    } sMotor_M3508_RxData_t;

    typedef struct sMotor_M6020_RxData_t
    {
        int16_t temp;               // 电机温度值
    } sMotor_M6020_RxData_t;

    typedef struct sMotor_Receive_Data_t
    {
        int16_t speed;           // 速度值
        int32_t angle;           // 编码器原始值
        int16_t current;         // 扭矩电流值
        uint32_t valid_mask;     // 有效数据掩码
        union {
            sMotor_M3508_RxData_t m3508;
            sMotor_M6020_RxData_t m6020;
            // sMotor_M2006_RxData_t m2006;
        } specific_data;         // 不同电机类型的特有数据
    } sMotor_Receive_Data_t;

    typedef enum motor_rx_valid_t
    {
        MOTOR_RX_VALID_CURRENT = 1u << 0,
        MOTOR_RX_VALID_TORQUE  = 1u << 1,
        MOTOR_RX_VALID_SPEED   = 1u << 2,
        MOTOR_RX_VALID_ANGLE   = 1u << 3,
        MOTOR_RX_VALID_TEMP    = 1u << 4,
    } motor_rx_valid_t;

    /**
     * @brief 掩码判断接收数据字段是否有效
     *
     * @param rx
     * @param mask
     * @return true
     * @return false
     */
    static inline bool motor_rx_has(const sMotor_Receive_Data_t *rx, uint32_t mask)
    {
        return (rx != NULL) && ((rx->valid_mask & mask) == mask);
    }

    typedef struct sMotor_data_t
    {
        uint8_t Tx_data[8];
        sMotor_Receive_Data_t Rx_data;
        sMotor_Heartbeat_Status_t heartbeat_status;
        void *interface_ptr;                            // 指向具体接口的指针
    } sMotor_data_t;

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
    typedef const sMotor_Receive_Data_t *(*motor_api_get_rxdata)(const struct device *dev);

    typedef int (*motor_api_update_serialized)(const struct device *dev, int16_t current);

    /**
     * @typedef motor_api_get_heartbeat_status
     * @brief application/middleware get motor heartbeat status
     *
     */
    typedef int (*motor_api_get_heartbeat_status)(const struct device *dev);

    typedef int (*motor_api_change_Tx_feq)(const struct device *dev, uint16_t new_feq);


    typedef struct motor_driver_api_t
    {
        motor_api_register register_motor;
        motor_api_get_rxdata get_rxdata;
        motor_api_change_Tx_feq change_Tx_feq;
        motor_api_get_heartbeat_status get_heartbeat_status;
        motor_api_update_serialized update_serialized;
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

    static inline const sMotor_Receive_Data_t *get_motor_rxdata(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->get_rxdata == NULL) {
            return NULL;
        }
        return api->get_rxdata(dev);
    }

    static inline int motor_update_serialized(const struct device *dev, int16_t current)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->update_serialized == NULL) {
            return -ENOSYS;
        }
        return api->update_serialized(dev, current);
    }

#ifdef __cplusplus
}
#endif
