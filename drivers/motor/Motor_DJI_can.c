/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_dji_can_motor

#include <drivers/motor.h>

#define LOG_LEVEL CONFIG_MOTOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_dji_can);

#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include <errno.h>
#include <string.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>

#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif
#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif

/* Fallbacks for static analysis (Zephyr builds define these via autoconf.h) */
#ifndef CONFIG_MOTOR_DJI_HEARTBEAT_OFFLINE_TIMEOUT_MS
#define CONFIG_MOTOR_DJI_HEARTBEAT_OFFLINE_TIMEOUT_MS 100
#endif
#ifndef CONFIG_MOTOR_DJI_HEARTBEAT_POLL_PERIOD_MS
#define CONFIG_MOTOR_DJI_HEARTBEAT_POLL_PERIOD_MS 10
#endif

/*
 * motor-id: DTS string -> const char*
 * control-mode: DTS enum -> DT_ENUM_IDX
 */
typedef struct motor_dji_cfg_t {
    uint16_t tx_id;
    uint16_t rx_id;
    int8_t motor_type;
    const char *motor_label;
    int8_t control_mode;
    uint16_t motor_encoder;
    uint8_t transmission_ratio;
    const struct device *can_dev;
#if defined(CONFIG_CAN_RX_MANAGER)
    const struct device *rx_mgr;            // 可选：接收管理器
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    const struct device *tx_mgr;            // 可选：发送管理器
#endif
} motor_dji_cfg_t;

typedef struct motor_dji_data_t
{
    sMotor_data_t motor_data;
    uint16_t Tx_feq;                        // 发送频率，单位Hz，0表示仅手动发送
    struct k_spinlock lock;                 // 保护 motor_data 的自旋锁，防止接收更新和心跳检测冲突
    bool registered;
#if defined(CONFIG_CAN_RX_MANAGER)
    int RxManager_slot_id;                  // CAN RX管理器 槽位ID
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    int TxManager_slot_id;                  // CAN TX管理器 槽位ID
#endif
#if defined(CONFIG_MOTOR_DJI_HEARTBEAT_AUTOCHECK)
    const struct device *dev_self;          // 指向自身设备的指针，用于心跳自动检测
    struct k_work_delayable hb_work;        // 心跳自动检测的延时工作
#endif
} motor_dji_data_t;

int motor_dji_update_heartbeat_status(const struct device *dev);

#if defined(CONFIG_MOTOR_DJI_HEARTBEAT_AUTOCHECK)
static void motor_dji_hb_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);           // 获取 k_work_delayable 指针
    motor_dji_data_t *data = CONTAINER_OF(dwork, motor_dji_data_t, hb_work);    // 获取 motor_dji_data_t 指针

    if (data->dev_self != NULL) {
        (void)motor_dji_update_heartbeat_status(data->dev_self);
        (void)k_work_schedule(&data->hb_work, K_MSEC(CONFIG_MOTOR_DJI_HEARTBEAT_POLL_PERIOD_MS));   // 重新调度下一次心跳检测
    }
}
#endif

/**
 * @brief 在不使用接收管理器的情况下，直接注册 CAN 接收回调函数，开始接收对应 ID 的 CAN 帧
 *
 * @param frame
 * @param user_data
 */
#if !defined(CONFIG_CAN_RX_MANAGER)
static void motor_dji_can_isr_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    const struct device *motor_dev = (const struct device *)user_data;
    if ((motor_dev == NULL) || (frame == NULL))
    {
        LOG_ERR("[dji_motor_err] rx handle Invalid arguments");
        return;
    }
    motor_dji_data_t *data = (motor_dji_data_t *)motor_dev->data;               // 获取 motor 数据结构体指针
    const motor_dji_cfg_t *cfg = (const motor_dji_cfg_t *)motor_dev->config;                        // 获取 motor 配置结构体指针
    if( cfg == NULL || data == NULL)
    {
        LOG_ERR("[dji_motor_err] data and cfg struct is NULL");
        return;
    }

    if ((frame->flags & CAN_FRAME_RTR) != 0U)
    {
        LOG_ERR("[dji_motor_err] RTR frame received");
        return;
    }

    data->motor_data.heartbeat_status.is_alive = true;
    /* 解析 CAN 帧数据 */
    switch (cfg->motor_type)
    {
    case 1: // MOTOR_DJI_TYPE_M3508
    {
        data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
        data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
        data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
        data->motor_data.Rx_data.specific_data.m3508.temp = (int16_t)frame->data[6];
        data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                         MOTOR_RX_VALID_SPEED |
                                                         MOTOR_RX_VALID_CURRENT |
                                                         MOTOR_RX_VALID_TEMP);
        break;
    }
    case 2: // MOTOR_DJI_TYPE_M2006
    {
        data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
        data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
        data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
        data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                         MOTOR_RX_VALID_SPEED |
                                                         MOTOR_RX_VALID_CURRENT);
        break;
    }
    case 3: // MOTOR_DJI_TYPE_M6020
    {
        data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
        data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
        data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
        data->motor_data.Rx_data.specific_data.m6020.temp = (int16_t)frame->data[6];
        data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                         MOTOR_RX_VALID_SPEED |
                                                         MOTOR_RX_VALID_CURRENT |
                                                         MOTOR_RX_VALID_TEMP);
        break;
    }
    case 0: // MOTOR_UNKNOWN
    default:
    {
        memset(&data->motor_data.Rx_data, 0, sizeof(data->motor_data.Rx_data));
        LOG_ERR("[dji_motor_err] rx handle unknown motor type: %d", cfg->motor_type);
        break;
    }
    }
    data->motor_data.heartbeat_status.heartbeat_tick = (uint64_t)k_uptime_get();
}
#endif


/**
 * @brief CAN 接收回调函数，在注册电机之后将会自动开始接收对应 ID 的 CAN 帧
 *
 * @param frame
 * @param user_data
 */
#if defined(CONFIG_CAN_RX_MANAGER)
static void motor_dji_can_rx_handler(const struct can_frame *frame, void *user_data)
{
    const struct device *motor_dev = (const struct device *)user_data;
    if ((motor_dev == NULL) || (frame == NULL))
    {
        LOG_ERR("[dji_motor_err] rx handle Invalid arguments");
        return;
    }
    motor_dji_data_t *data = (motor_dji_data_t *)motor_dev->data;             // 获取 motor 数据结构体指针
    const motor_dji_cfg_t *cfg = (const motor_dji_cfg_t *)motor_dev->config;        // 获取 motor 配置结构体指针
    if (cfg == NULL || data == NULL)
    {
        LOG_ERR("[dji_motor_err] data and cfg struct is NULL");
        return;
    }

    /* manager 已默认跳过 RTR，这里再做一次保护 */
    if ((frame->flags & CAN_FRAME_RTR) != 0U) {
        LOG_ERR("[dji_motor_err] RTR frame received");
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);           // 加锁保护 motor_data
    data->motor_data.heartbeat_status.is_alive = true;
    /* 解析 CAN 帧数据 */
    switch (cfg->motor_type)
    {
        case 1: // MOTOR_DJI_TYPE_M3508
        {
            data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
            data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
            data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
            data->motor_data.Rx_data.specific_data.m3508.temp = (int16_t)frame->data[6];
            data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                            MOTOR_RX_VALID_SPEED |
                                                            MOTOR_RX_VALID_CURRENT |
                                                            MOTOR_RX_VALID_TEMP);
            break;
        }
        case 2: // MOTOR_DJI_TYPE_M2006
        {
            data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
            data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
            data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
            data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                            MOTOR_RX_VALID_SPEED |
                                                            MOTOR_RX_VALID_CURRENT);
            break;
        }
        case 3: // MOTOR_DJI_TYPE_M6020
        {
            data->motor_data.Rx_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
            data->motor_data.Rx_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
            data->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
            data->motor_data.Rx_data.specific_data.m6020.temp = (int16_t)frame->data[6];
            data->motor_data.Rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ANGLE |
                                                            MOTOR_RX_VALID_SPEED |
                                                            MOTOR_RX_VALID_CURRENT |
                                                            MOTOR_RX_VALID_TEMP);
            break;
        }
        case 0: // MOTOR_UNKNOWN
        default:
        {
            memset(&data->motor_data.Rx_data, 0, sizeof(data->motor_data.Rx_data));
            LOG_ERR("[dji_motor_err] rx handle unknown motor type: %d", cfg->motor_type);
            break;
        }
    }
    data->motor_data.heartbeat_status.heartbeat_tick = (uint64_t)k_uptime_get();       // 更新心跳时间戳
    k_spin_unlock(&data->lock, key);                           // 解锁
}
#endif


/**
 * @brief
 * @param frame
 * @param user_data
 * @return int
 */
#if defined(CONFIG_CAN_TX_MANAGER)
static int motor_dji_can_tx_fillbuffer_handler(struct can_frame *frame, void *user_data)
{
    const struct device *motor = (const struct device *)user_data; // 还原 motor 指针
    if ((motor == NULL) || (frame == NULL))
    {
        LOG_ERR("[dji_motor_err] tx handle Invalid arguments");
        return -EINVAL;
    }
    motor_dji_data_t *data = (motor_dji_data_t *)motor->data;
    const motor_dji_cfg_t *cfg = (const motor_dji_cfg_t *)motor->config;
    if (data == NULL || cfg == NULL)
    {
        LOG_ERR("[dji_motor_err] tx handle data or cfg NULL");
        return -EINVAL;
    }
    // LOG_INF("motor tx fillbuffer handler called, control_mode=%d, tx_id=%03X", cfg->control_mode, cfg->tx_id);
    switch (cfg->control_mode)
    {
        case 0: // torque
        case 1: // velocity
        {
            frame->dlc = 8;
            frame->flags = 0;
            k_spinlock_key_t key = k_spin_lock(&data->lock);
            int diff = (int)cfg->rx_id - (int)cfg->tx_id;
            if (diff <= 0 || diff > 8) {
                LOG_ERR("[dji_motor_err] tx handle invalid id difference: tx_id=%d, rx_id=%d", cfg->tx_id, cfg->rx_id);
                k_spin_unlock(&data->lock, key);
                return -EINVAL;
            }
            int off = (diff > 4) ? (diff - 4) : diff;
            if (off <= 0 || off > 4) {
                LOG_ERR("[dji_motor_err] tx handle computed off out of range: %d", off);
                k_spin_unlock(&data->lock, key);
                return -EINVAL;
            }
            int idx = 2 * (off - 1);
            if ((idx + 2) > 8) {
                LOG_ERR("[dji_motor_err] tx handle target index overflow: idx=%d", idx);
                k_spin_unlock(&data->lock, key);
                return -EFAULT;
            }
            memcpy(&frame->data[idx], &data->motor_data.Tx_data[0], 2);
            k_spin_unlock(&data->lock, key);
            return 0;
        }
        default:
        {
            LOG_ERR("[dji_motor_err] tx handle unknown control mode: %d", cfg->control_mode);
            return -EINVAL;
        }
    }
    return -EINVAL; // should not reach here
    return 0;
}
#endif

/**
 * @brief 暴露给上层的电机控制接口函数，负责将 current 值序列化到 Tx_data 里，电机发送报文的协议就在这里体现了。
 *
 * @param dev
 * @param current
 * @return int
 */
static int motor_dji_can_control(const struct device *dev, int16_t current)
{
    if(dev == NULL)
    {
        LOG_ERR("[dji_motor_err] update serialized Invalid arguments");
        return -EINVAL;
    }
    motor_dji_data_t *data = dev->data;
    const motor_dji_cfg_t *cfg = dev->config;
    if (data == NULL || cfg == NULL)
    {
        LOG_ERR("[dji_motor_err] update serialized data or cfg NULL");
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    switch(cfg->control_mode)
    {
        case 0: // torque
        case 1: // velocity
            data->motor_data.Tx_data[0] = (uint8_t)((current >> 8) & 0xFF);
            data->motor_data.Tx_data[1] = (uint8_t)(current & 0xFF);
            break;
        default:
            k_spin_unlock(&data->lock, key);
            return -EINVAL;
    }
    k_spin_unlock(&data->lock, key);
    return 0;
}


/**
 * @brief dji motor 心跳状态更新函数，供应用层/中间件调用以获取最新心跳状态.
 *        这个函数既可以在应用层创建线程调用，也可以启动自动检测（CONFIG_MOTOR_DJI_HEARTBEAT_AUTOCHECK）
 *
 * @param dev
 * @return int
 */
int motor_dji_update_heartbeat_status(const struct device *dev)
{
    if (dev == NULL) {
        LOG_ERR("[dji_motor_err] update heartbeat Invalid arguments");
        return -EINVAL;
    }

    motor_dji_data_t *data = dev->data;
    if (data == NULL) {
        LOG_ERR("[dji_motor_err] update heartbeat data NULL");
        return -EINVAL;
    }

    const motor_dji_cfg_t *cfg = (const motor_dji_cfg_t *)data->motor_data.interface_ptr;
    uint64_t current_tick = (uint64_t)k_uptime_get();

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    uint64_t last_tick = data->motor_data.heartbeat_status.heartbeat_tick;
    bool prev_alive = data->motor_data.heartbeat_status.is_alive;

    /* 尚未收到过任何帧时，不做掉线告警；Rx_data 默认保持为 0 */
    if (last_tick == 0U) {
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    uint64_t elapsed = current_tick - last_tick;

    /* 如果超过阈值没有收到心跳，则认为电机掉线：清零接收值并在离线边沿告警一次 */
    if (elapsed > (uint64_t)CONFIG_MOTOR_DJI_HEARTBEAT_OFFLINE_TIMEOUT_MS) {
        data->motor_data.heartbeat_status.is_alive = false;

        /* 只有从在线->离线时，才清零并告警；避免每次轮询刷屏 */
        if (prev_alive) {
            memset(&data->motor_data.Rx_data, 0, sizeof(data->motor_data.Rx_data));
            LOG_ERR("[dji_motor_err] motor offline (%s, rx=0x%03x): no CAN frames for %llu ms",
                    (cfg != NULL && cfg->motor_label != NULL) ? cfg->motor_label : "unknown",
                    (cfg != NULL) ? (unsigned int)cfg->rx_id : 0U,
                    (unsigned long long)elapsed);
        }
    } else {
        /* 心跳在窗口内：确保在线 */
        data->motor_data.heartbeat_status.is_alive = true;
    }

    k_spin_unlock(&data->lock, key);

    return 0;
}

/**
 * @brief 单电机注册接口，通过 motor_id 识别电机实例，配置了can的接收过滤器
 *
 * @param dev
 * @return int
 */
static int motor_dji_can_register_motor(const struct device *dev)
{
    const motor_dji_cfg_t *cfg = dev->config;
    motor_dji_data_t *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        LOG_ERR("[dji_motor_err] register Invalid arguments");
        return -EINVAL;
    }

    if (!device_is_ready(cfg->can_dev)) {
        LOG_ERR("[dji_motor_err] CAN device not ready");
        return -ENODEV;
    }

    if (data->registered)
    {
        LOG_WRN("[dji_motor_err] motor already registered,please check your code");
        return -EALREADY;
    }
#if defined(CONFIG_CAN_RX_MANAGER)
    if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
        LOG_ERR("[dji_motor_err] RX manager not ready");
        return -ENODEV;
    }
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
        LOG_ERR("[dji_motor_err] TX manager not ready");
        return -ENODEV;
    }
#endif

    // can 过滤器
    struct can_filter filter = {
        .id = cfg->rx_id & CAN_STD_ID_MASK,
        .mask = CAN_STD_ID_MASK,
        .flags = 0,
    };
    int rx_ret = -1;

#if defined(CONFIG_CAN_RX_MANAGER) // 将电机接收交给 CAN RX 管理器处理
    rx_ret = can_rx_manager_register(cfg->rx_mgr, &filter, motor_dji_can_rx_handler, (void *)dev);
    if (rx_ret < 0) {
        LOG_ERR("[dji_motor_err] Failed to register motor on RxManager: %d", rx_ret);
        return rx_ret;
    }
    else LOG_INF("Motor (%s) registered on RxManager, CAN RX ID: 0x%03X  slotID: %d", cfg->motor_label, cfg->rx_id, rx_ret);
    data->RxManager_slot_id = rx_ret;
#else // 不使用 RX 管理器，直接注册 CAN 接收回调函数
    rx_ret = can_add_rx_filter(cfg->can_dev, motor_dji_can_isr_cb, (void *)dev, &filter);
    if (rx_ret < 0) {
        LOG_ERR("[dji_motor_err] Failed to add CAN RX filter, error: %d", rx_ret);
        return rx_ret;
    }
    else LOG_INF("Motor (%s) registered CAN RX ID: 0x%03X", cfg->motor_label, cfg->rx_id);
#endif


#if defined(CONFIG_CAN_TX_MANAGER)
    int tx_ret = -1;
    tx_ret = can_tx_manager_register(cfg->tx_mgr, cfg->tx_id, cfg->rx_id, 8, 0, data->Tx_feq,
                                        motor_dji_can_tx_fillbuffer_handler, (void *)dev);
    if (tx_ret < 0) {
        LOG_ERR("[dji_motor_err] Failed to register CAN TX filter: %d", tx_ret);
        return tx_ret;
    }
    else LOG_INF("Motor (%s) registered on TxManager, CAN TX ID: 0x%03X  slotID: %d", cfg->motor_label, cfg->tx_id, tx_ret);
    data->TxManager_slot_id = tx_ret;
#else
    LOG_INF("Motor (%s) did not register on TxManager, CAN TX ID: 0x%03X", cfg->motor_label, cfg->tx_id);
#endif

    data->registered = true;
    data->motor_data.Tx_data[0] = 0;
    data->motor_data.interface_ptr = (void *)cfg;
    data->motor_data.Rx_data.valid_mask = 0U;
    data->motor_data.heartbeat_status.is_alive = false;
    data->motor_data.heartbeat_status.heartbeat_tick = 0;
    return 0;
}

/**
 * @brief 暴露给中间件获取电机数据的接口，Atention!!!!!:
 *        这里直接返回了 Rx_data 的指针，上层不可更改
 *        此外因为大部分mcu都是单核的，主线程和中断不会并发执行，所以是安全的
 *        如果在多核平台上使用，请自行加锁保护！！！！！或者改成双缓冲及快照
 *
 * @param dev
 * @return const sMotor_Receive_Data_t*
 */
static const sMotor_Receive_Data_t *motor_dji_can_get_rxdata(const struct device *dev)
{
    motor_dji_data_t *data = dev->data;
    if (data == NULL) {
        LOG_WRN("[dji_motor_err] get_rxdata dev NULL");
        return NULL;
    }

    return &data->motor_data.Rx_data;
}


/**
 * @brief 暴露给中间件的修改发送频率接口，修改后会立即生效
 *
 * @param dev
 * @param new_feq
 * @return int
 */
static int motor_dji_can_change_Tx_feq(const struct device *dev, uint16_t new_feq)
{
    motor_dji_data_t *data = dev->data;
    if (data == NULL) {
        LOG_ERR("[dji_motor_err] change Tx feq Invalid arguments");
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->Tx_feq = new_feq;
    k_spin_unlock(&data->lock, key);
    LOG_WRN("[dji_motor] Tx frequency changed to %u Hz", new_feq);
    return 0;
}

/**
 * @brief 获取电机心跳状态接口，供应用层/中间件调用
 *
 * @param dev
 * @return int 1: alive, 0: not alive, <0: error code
 */
static int motor_dji_can_get_heartbeat_status(const struct device *dev)
{
    int ret = motor_dji_update_heartbeat_status(dev);
    if (ret < 0) {
        return ret;
    }

    motor_dji_data_t *data = dev->data;
    if (data == NULL) {
        return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    bool alive = data->motor_data.heartbeat_status.is_alive;
    k_spin_unlock(&data->lock, key);

    return alive ? 1 : 0;
}

static const motor_driver_api_t motor_dji_can_api = {
    .register_motor = motor_dji_can_register_motor,
    .change_Tx_feq = motor_dji_can_change_Tx_feq,
    .update_serialized = motor_dji_can_control,
    .get_heartbeat_status = motor_dji_can_get_heartbeat_status,
    .get_rxdata = motor_dji_can_get_rxdata,
};

/**
 * @brief dji电机实例的初始化
 *
 * @param dev
 * @return int
 */
static int motor_dji_can_init(const struct device *dev)
{
    const motor_dji_cfg_t *cfg = dev->config;
    motor_dji_data_t *data = dev->data;

    if (!device_is_ready(cfg->can_dev)) {
        return -ENODEV;
    }

    int start_ret = can_start(cfg->can_dev);
    if ((start_ret < 0) && (start_ret != -EALREADY))
    {
        LOG_ERR("[dji_motor_err] Failed to start CAN device, error: %d", start_ret);
        return start_ret;
    }

#if defined(CONFIG_CAN_RX_MANAGER)
    if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
        __ASSERT(false, "[dji_motor_err] RX manager not ready for device %s", dev->name);
        return -ENODEV;
    }
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
        __ASSERT(false, "[dji_motor_err] TX manager not ready for device %s", dev->name);
        return -ENODEV;
    }
#endif

    data->registered = false;
    memset(&data->motor_data, 0, sizeof(data->motor_data));
    data->motor_data.interface_ptr = (void *)cfg;
    data->motor_data.Rx_data.valid_mask = 0U;
    data->motor_data.heartbeat_status.is_alive = false;
    data->motor_data.heartbeat_status.heartbeat_tick = 0;

#if defined(CONFIG_MOTOR_DJI_HEARTBEAT_AUTOCHECK)
    data->dev_self = dev;
    k_work_init_delayable(&data->hb_work, motor_dji_hb_work_handler);
    (void)k_work_schedule(&data->hb_work, K_MSEC(CONFIG_MOTOR_DJI_HEARTBEAT_POLL_PERIOD_MS));
#endif

    return 0;
}


/* ---------- Devicetree helpers ---------- */

/* 获取 control-mode enum 的索引；未配置则返回 -1 */
#define MOTOR_DJI_CONTROL_MODE(inst) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, control_mode), (DT_INST_ENUM_IDX(inst, control_mode)), (-1))

#define MOTOR_DJI_TYPE(inst) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, motor_type), (DT_INST_ENUM_IDX(inst, motor_type)), (-1))

#define MOTOR_DJI_DEFINE(inst) \
    static const motor_dji_cfg_t motor_dji_cfg_##inst = { \
        .tx_id = (uint16_t)DT_INST_PROP(inst, tx_id), \
        .rx_id = (uint16_t)DT_INST_PROP(inst, rx_id), \
        .motor_label = DT_INST_PROP(inst, label), \
        .motor_type = (int8_t)MOTOR_DJI_TYPE(inst), \
        .control_mode = (int8_t)MOTOR_DJI_CONTROL_MODE(inst), \
        .motor_encoder = (uint16_t)DT_INST_PROP(inst, motor_encoder), \
        .transmission_ratio = (uint8_t)DT_INST_PROP(inst, motor_transmission_ratio), \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)), \
        IF_ENABLED(CONFIG_CAN_RX_MANAGER, ( \
            .rx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rx_manager), \
                          (DEVICE_DT_GET(DT_INST_PHANDLE(inst, rx_manager))), (NULL)), \
        )) \
        IF_ENABLED(CONFIG_CAN_TX_MANAGER, ( \
            .tx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, tx_manager), \
                          (DEVICE_DT_GET(DT_INST_PHANDLE(inst, tx_manager))), (NULL)), \
        )) \
    }; \
    static motor_dji_data_t motor_dji_data_##inst = {  \
        .Tx_feq = (uint16_t)DT_INST_PROP(inst, tx_feq), \
    };  \
    DEVICE_DT_INST_DEFINE(inst, motor_dji_can_init, NULL, &motor_dji_data_##inst, \
                      &motor_dji_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
                  &motor_dji_can_api);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(MOTOR_DJI_DEFINE)
#endif
