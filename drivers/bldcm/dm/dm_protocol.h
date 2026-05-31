/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef DM_PROTOCOL_H
#define DM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/bldcm/bldcm_dm.h>
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

#define LOG_LEVEL CONFIG_BLDCM_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_dm_can);


/* Fallbacks for static analysis (Zephyr builds define these via autoconf.h) */
#ifndef CONFIG_BLDCM_HEARTBEAT_OFFLINE_TIMEOUT_MS
#define CONFIG_BLDCM_HEARTBEAT_OFFLINE_TIMEOUT_MS 100
#endif
#ifndef CONFIG_BLDCM_HEARTBEAT_POLL_PERIOD_MS
#define CONFIG_BLDCM_HEARTBEAT_POLL_PERIOD_MS 10
#endif

#define DM43_MAX_CURRENT 10.25  // dm43最大相电流，单位A
#define DM60_MAX_CURRENT 20.5   // dm60最大相电流，单位A
#define DM80_MAX_CURRENT 41     // dm80最大相电流，单位A
#define DM100_MAX_CURRENT 100   // dm100最大相电流，单位A

typedef struct dm_param_t
{
    float vel_max;
    float vel_min;
    float tq_max;
    float tq_min;
    float pos_max;
    float pos_min;
    float kp_max;
    float kp_min;
    float kd_max;
    float kd_min;
} dm_param_t;

/*
 * motor-id: DTS string -> const char*
 * control-mode: DTS enum -> DT_ENUM_IDX
 */
typedef struct motor_dm_cfg_t {
    uint16_t tx_id;
    uint16_t rx_id;
    int8_t motor_type;
    const char *motor_label;
    int8_t control_mode;
    uint16_t motor_encoder;
    uint8_t transmission_ratio;
    const dm_param_t param_limit;           // 上位机设定的参数限幅，必须要和固件对应起来！！！
    const struct device *can_dev;
#if defined(CONFIG_CAN_RX_MANAGER)
    const struct device *rx_mgr;            // 可选：接收管理器
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
    const struct device *tx_mgr;            // 可选：发送管理器
#endif
} motor_dm_cfg_t;


typedef struct motor_dm_data_t
{
    smotor_data_t motor_data;
    rw_reg_t reg_info;
    uint16_t Tx_feq;                        // 发送频率，单位Hz，0表示仅手动发送
    struct k_spinlock lock;                 // 保护 motor_data 的自旋锁，防止接收更新和心跳检测冲突
    bool registered;
#if defined(CONFIG_CAN_RX_MANAGER)
    int rxmanager_slot_id;                  // CAN RX管理器 槽位ID
#endif

#if defined(CONFIG_BLDCM_HEARTBEAT_AUTOCHECK)
    const struct device *dev_self;          // 指向自身设备的指针，用于心跳自动检测
    struct k_work_delayable hb_work;        // 心跳自动检测的延时工作
#endif
} motor_dm_data_t;


typedef enum {
    DM_CMD_RW_REG_ID = 0x7FF,                  // 达妙读写寄存器的报文ID
    DM_CMD_R_REG = 0x33,                      // 读寄存器命令字
    DM_CMD_W_REG = 0x55,                      // 写寄存器命令字
    DM_CMD_STORE = 0xAA,                        // 存储命令字
} dm_cmd_id_e;


static inline float uint_to_float(uint16_t x_uint, float xmin, float xmax, uint8_t bit)
{
    float span = xmax - xmin;
    float data_norm = (float)x_uint / ((1 << bit) - 1);
    float x_float = data_norm * span + xmin;
    return x_float;
}

static inline uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bit)
{
    if (x < xmin) {
        x = xmin;
    } else if (x > xmax) {
        x = xmax;
    }
    float span = xmax - xmin;
    float data_norm = (x - xmin) / span;
    uint16_t x_uint = (uint16_t)(data_norm * ((1 << bit) - 1));
    return x_uint;
}

/*---------------------------------------------------dm_receive_protocol----------------------------------------------------- */

static inline int motor_dm_rev_data(struct device *dev, const struct can_frame *frame)
{
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(data == NULL || frame == NULL || cfg == NULL) {
        LOG_ERR("[dm_motor_err] Invalid arguments to motor_dm_revdata");
        return -EINVAL;
    }

    data->motor_data.rx_data.specific_data.dm.errState = (uint8_t)(frame->data[0] >> 4);
    data->motor_data.rx_data.encoder = (uint16_t)(frame->data[1] | (frame->data[2] << 8));
    data->motor_data.rx_data.speed = (int16_t)(frame->data[3] << 4 | frame->data[4] >> 4);
    data->motor_data.rx_data.iq = (int16_t)(((frame->data[4] & 0x0F) << 8) | frame->data[5]);
    data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_IQ | 
                                                     MOTOR_RX_VALID_SPEED | 
                                                     MOTOR_RX_VALID_ENCODER | 
                                                     MOTOR_DM);
    data->motor_data.rx_data.specific_data.dm.vel_real = 
        uint_to_float(data->motor_data.rx_data.speed, cfg->param_limit.vel_min, cfg->param_limit.vel_max, 12);
    data->motor_data.rx_data.specific_data.dm.pos_real = 
        uint_to_float(data->motor_data.rx_data.encoder, 0, cfg->param_limit.pos_max, 16);
    data->motor_data.rx_data.specific_data.dm.iq_real = 
        uint_to_float(data->motor_data.rx_data.iq, cfg->param_limit.tq_min, cfg->param_limit.tq_max, 12);
    return 0;
}

/**
 * @brief 达妙电机读取寄存器返回数据的解析函数
 * 
 * @param frame 
 * @return rw_reg_t* 
 */
static inline bool motor_dm_readreg_back(rw_reg_t *reg_data, const struct can_frame *frame)
{
    if (reg_data == NULL || frame == NULL) {
        LOG_ERR("[dm_motor_err] Invalid arguments to readreg back function");
        return false;
    }
    if(frame->data[2] != DM_CMD_R_REG)
    {
        LOG_ERR("[dm_motor_err] Received CAN frame is not a read register response");
        return false;
    }
    reg_data->CANID = frame->data[0] | (frame->data[1] << 8);
    
    reg_data->reg_addr = frame->data[3];
    reg_data->data[0] = frame->data[4];
    reg_data->data[1] = frame->data[5];
    reg_data->data[2] = frame->data[6];
    reg_data->data[3] = frame->data[7];
    return true;
}

/**
 * @brief 达妙电机写寄存器返回数据的解析函数
 * 
 * @param frame 
 * @return rw_reg_t* 
 */
static inline bool motor_dm_writereg_back(rw_reg_t *reg_data, const struct can_frame *frame)
{
    if (reg_data == NULL || frame == NULL) {
        LOG_ERR("[dm_motor_err] Invalid arguments to writereg back function");
        return false;
    }
    if(frame->data[2] != DM_CMD_W_REG)
    {
        LOG_ERR("[dm_motor_err] Received CAN frame is not a write register response");
        return false;
    }
    reg_data->CANID = frame->data[0] | (frame->data[1] << 8);
    
    reg_data->reg_addr = frame->data[3];         // 注意这里返回的是寄存器设置的值，不是实际值
    reg_data->data[0] = frame->data[4];  
    reg_data->data[1] = frame->data[5];
    reg_data->data[2] = frame->data[6];
    reg_data->data[3] = frame->data[7];
    return true;
}

/**
 * @brief 达妙电机存储寄存器返回数据的解析函数
 * 
 * @param frame 
 * @return void 
 */
static inline void motor_dm_storereg_back(const struct can_frame *frame)
{
    if (frame == NULL) {
        LOG_ERR("[dm_motor_err] Invalid arguments to storereg back function");
        return;
    }
    uint16_t canID = frame->data[0] | (frame->data[1] << 8);
    if(frame->data[3] == 1 && frame->data[2] == DM_CMD_STORE)
    {
        LOG_INF("[dm_motor_info] Store register successfully, motor ID: 0x%04X", canID);
        return;
    }
    LOG_ERR("[dm_motor_err] Failed to store register, motor ID: 0x%04X", canID);
    return;
}

/*---------------------------------------------------dm_single_control_api----------------------------------------------------- */


static void motor_dm_one_shot_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

/**
 * @brief 达妙电机读寄存器的数据
 * 
 * @param dev 
 * @param reg_addr 寄存器的地址
 * @return int 
 */
static inline int motor_dm_read_register(const struct device *dev, uint8_t reg_addr)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.flags = 0;
    item.dlc = 8;
    item.id = DM_CMD_RW_REG_ID;     // 达妙读写寄存器的报文ID
    item.data[0] = (cfg->tx_id) & 0xFF;             // CANID的低8位
    item.data[1] = ((cfg->tx_id) >> 8) & 0xFF;    // CANID的高8位
    item.data[2] = DM_CMD_R_REG;                          // 读取寄存器命令字
    item.data[3] = reg_addr;                      // 寄存器地址

    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send read register command: %d", ret);
    }
    return ret;
}

/**
 * @brief 写入寄存器数据
 * 
 * @param dev 
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return int 
 */
static inline int motor_dm_write_register(const struct device *dev, uint8_t reg_addr, uint32_t data)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.flags = 0;
    item.dlc = 8;
    item.id = DM_CMD_RW_REG_ID;     // 达妙读写寄存器的报文ID
    item.data[0] = (cfg->tx_id) & 0xFF;             // CANID的低8位
    item.data[1] = ((cfg->tx_id) >> 8) & 0xFF;    // CANID的高8位
    item.data[2] = DM_CMD_W_REG;                          // 写寄存器命令字
    item.data[3] = reg_addr;                      // 寄存器地址
    item.data[4] = (data) & 0xFF;           
    item.data[5] = (data >> 8) & 0xFF;
    item.data[6] = (data >> 16) & 0xFF;
    item.data[7] = (data >> 24) & 0xFF;
    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send write register command: %d", ret);
    }
    return ret;
}

/**
 * @brief 存储寄存器数据到达妙电机的非易失性存储中
 * 
 * @param dev 
 * @param reg_addr 
 * @return int 
 */
static inline int motor_dm_store(const struct device *dev, uint8_t reg_addr)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.flags = 0;
    item.dlc = 4;
    item.id = DM_CMD_RW_REG_ID;     // 达妙存储寄存器的报文ID
    item.data[0] = (cfg->tx_id) & 0xFF;             // CANID的低8位
    item.data[1] = (cfg->tx_id >> 8) & 0xFF;    // CANID的高8位
    item.data[2] = DM_CMD_STORE;                          // 存储命令字
    item.data[3] = reg_addr;                      // 寄存器地址
    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send store command: %d", ret);
    }
    return ret;
}


/**
 * @brief 达妙电机使能函数
 * 
 * @param dev 
 * @return int 
 */
static inline int motor_dm_enable(const struct device *dev)
{
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device data or config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.id = cfg->tx_id;
    item.flags = 0;
    item.dlc = 8;
    item.data[0] = 0xFF;
    item.data[1] = 0xFF;
    item.data[2] = 0xFF;
    item.data[3] = 0xFF;
    item.data[4] = 0xFF;
    item.data[5] = 0xFF;
    item.data[6] = 0xFF;
    item.data[7] = 0xFC;     // 达妙电机使能命令
    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send enable command: %d", ret);
    }
    return ret;
}

/**
 * @brief 达妙电机失能函数
 * 
 * @param dev 
 * @return int 
 */
static inline int motor_dm_disable(const struct device *dev)
{
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device data or config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.id = cfg->tx_id;
    item.flags = 0;
    item.dlc = 8;
    item.data[0] = 0xFF;
    item.data[1] = 0xFF;
    item.data[2] = 0xFF;
    item.data[3] = 0xFF;
    item.data[4] = 0xFF;
    item.data[5] = 0xFF;
    item.data[6] = 0xFF;
    item.data[7] = 0xFD;     // 达妙电机失能命令
    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send disable command: %d", ret);
    }
    return ret;
}

/**
 * @brief 达妙电机保存零点的函数
 * 
 * @param dev 
 * @return int 
 */
static inline int motor_dm_save_zero_pos(const struct device *dev)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device data or config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.id = cfg->tx_id;
    item.flags = 0;
    item.dlc = 8;
    item.data[0] = 0xFF;
    item.data[1] = 0xFF;
    item.data[2] = 0xFF;
    item.data[3] = 0xFF;
    item.data[4] = 0xFF;
    item.data[5] = 0xFF;
    item.data[6] = 0xFF;
    item.data[7] = 0xFE;     // 达妙电机保存

    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send save zero command: %d", ret);
    }
    return ret;
}

/**
 * @brief 达妙电机清楚错误的函数
 * 
 * @param dev 
 * @return int 
 */
static inline int motor_dm_clear_error(const struct device *dev)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    if(cfg == NULL)
    {
        LOG_ERR("[dm_motor_err] Device data or config is NULL");
        return -EINVAL;
    }
    struct can_frame item = {0};
    item.id = cfg->tx_id;
    item.flags = 0;
    item.dlc = 8;
    item.data[0] = 0xFF;
    item.data[1] = 0xFF;
    item.data[2] = 0xFF;
    item.data[3] = 0xFF;
    item.data[4] = 0xFF;
    item.data[5] = 0xFF;
    item.data[6] = 0xFF;
    item.data[7] = 0xFB;     // 达妙电机清除错误

    int ret = can_send(cfg->can_dev, &item, K_NO_WAIT, motor_dm_one_shot_cb, NULL);
    if (ret < 0) {
        LOG_ERR("[dm_motor_err] Failed to send clear error command: %d", ret);
    }
    return ret;
}


/**
 * @brief MIT控制模式。建议看达妙电机的相关文档，理解MIT控制模式的原理和参数含义后再使用这个函数进行控制。
 * 
 * @param dev 
 * @param pos 
 * @param vel 
 * @param Kp 
 * @param Kd 
 * @param Tq 
 * @return int 
 */
static inline int motor_dm_mit_ctrl(const struct device *dev, float pos, float vel, float Kp, float Kd, float Tq)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    if(cfg == NULL || data == NULL)
    {
        LOG_ERR("[dm_motor_err] Device config or data is NULL");
        return -EINVAL;
    }

    data->motor_data.tx_data[0] = (float_to_uint(pos, cfg->param_limit.pos_min, cfg->param_limit.pos_max, 16) >> 8) & 0xFF;
    data->motor_data.tx_data[1] = float_to_uint(pos, cfg->param_limit.pos_min, cfg->param_limit.pos_max, 16) & 0xFF;
    data->motor_data.tx_data[2] = (float_to_uint(vel, cfg->param_limit.vel_min, cfg->param_limit.vel_max, 12) >> 4) & 0xFF;
    data->motor_data.tx_data[3] = ((float_to_uint(vel, cfg->param_limit.vel_min, cfg->param_limit.vel_max, 12) & 0x0F) << 4) 
                                | ((float_to_uint(Kp, cfg->param_limit.kp_min, cfg->param_limit.kp_max, 12) >> 8) & 0x0F);
    data->motor_data.tx_data[4] = float_to_uint(Kp, cfg->param_limit.kp_min, cfg->param_limit.kp_max, 12) & 0xFF;
    data->motor_data.tx_data[5] = (float_to_uint(Kd, cfg->param_limit.kd_min, cfg->param_limit.kd_max, 12) >> 4) & 0xFF;
    data->motor_data.tx_data[6] = ((float_to_uint(Kd, cfg->param_limit.kd_min, cfg->param_limit.kd_max, 12) & 0x0F) << 4) 
                                | ((float_to_uint(Tq, cfg->param_limit.tq_min, cfg->param_limit.tq_max, 12) >> 8) & 0x0F);
    data->motor_data.tx_data[7] = float_to_uint(Tq, cfg->param_limit.tq_min, cfg->param_limit.tq_max, 12) & 0xFF;
    return 0;
}

/**
 * @brief 达妙电机位置速度控制函数
 * 
 * @param dev 
 * @param pos 
 * @param vel 这个速度是梯形加速度运行下的最高速度，也就是匀速段的速度值。
 * @return int 
 */
static inline int motor_dm_posvel_ctrl(const struct device *dev, float pos, float vel)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    if(cfg == NULL || data == NULL)
    {
        LOG_ERR("[dm_motor_err] Device config or data is NULL");
        return -EINVAL;
    }  

    uint8_t *pbuf,*vbuf;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;
    data->motor_data.tx_data[0] = *pbuf;
    data->motor_data.tx_data[1] = *(pbuf + 1);
    data->motor_data.tx_data[2] = *(pbuf + 2);
    data->motor_data.tx_data[3] = *(pbuf + 3);
    data->motor_data.tx_data[4] = *vbuf;
    data->motor_data.tx_data[5] = *(vbuf + 1);
    data->motor_data.tx_data[6] = *(vbuf + 2);
    data->motor_data.tx_data[7] = *(vbuf + 3);
    return 0;

}

/**
 * @brief 达妙电机速度控制模式，需要注意这个报文的dlc应该是4
 * 
 * @param dev 
 * @param vel 
 * @return int 
 */
static inline int motor_dm_vel_ctrl(const struct device *dev, float vel)
{
    motor_dm_cfg_t *cfg = (motor_dm_cfg_t *)dev->config;
    motor_dm_data_t *data = (motor_dm_data_t *)dev->data;
    if(cfg == NULL || data == NULL)
    {
        LOG_ERR("[dm_motor_err] Device config or data is NULL");
        return -EINVAL;
    }  

    uint8_t *vbuf;
    vbuf = (uint8_t *)&vel;
    data->motor_data.tx_data[0] = *vbuf;
    data->motor_data.tx_data[1] = *(vbuf + 1);
    data->motor_data.tx_data[2] = *(vbuf + 2);
    data->motor_data.tx_data[3] = *(vbuf + 3);
    return 0;

}

#ifdef __cplusplus
}
#endif

#endif