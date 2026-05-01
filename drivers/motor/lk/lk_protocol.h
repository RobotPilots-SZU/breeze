#ifndef LK_PROTOCOL_H
#define LK_PROTOCOL_H

#include <drivers/motor/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/can.h>

#define LOG_LEVEL CONFIG_MOTOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_lk_can);

#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif
#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif

#ifndef CONFIG_MOTOR_HEARTBEAT_OFFLINE_TIMEOUT_MS
#define CONFIG_MOTOR_HEARTBEAT_OFFLINE_TIMEOUT_MS 100
#endif
#ifndef CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS
#define CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS 10
#endif

#define MGMF_CURRENT_MAX 2000
#define MS_CURRENT_MAX 850
#define SPEEDVALUE_MAX 32768
#define POSITVALUE_MAX 32768

// 单电机控制模式ID定义，具体含义请参考电机协议文档
typedef enum {
	READ_MOTOR_STATE1 = 0x9A,
	READ_MOTOR_STATE2 = 0x9C,
	READ_MOTOR_STATE3 = 0x9D,
	READ_BRAKE_STATE = 0x8C,
	READ_CONTROL_PARAM = 0xC0,
	READ_ENCODER = 0x90,
	READ_MULTI_ENCODER = 0x92,
	READ_SINGLE_ENCODER = 0x94,
	SET_ENCODER_ZERO = 0x19,
	SET_ENCODER_SELF = 0x95, // 设置当前位置为任意角度
	WRITE_CONTROL_PARAM = 0xC1,
	CLEAR_MOTOR_ERROR = 0x9B,
	DISABLE_MOTOR = 0x80,
	ENABLE_MOTOR = 0x88,
	STOP_MOTOR = 0x81,
	OPEN_LOOP_CONTROL = 0xA0, // only in MS
	CLOSED_LOOP_CONTROL = 0xA1,
	CLOSED_SPEED_CONTROL = 0xA2,
	MULTI_POSITION_CONTROL1 = 0xA3,
	MULTI_POSITION_CONTROL2 = 0xA4,
	SINGLE_POSITION_CONTROL1 = 0xA5,
	SINGLE_POSITION_CONTROL2 = 0xA6,
	INCRE_POSITION_CONTROL1 = 0xA7,
	INCRE_POSITION_CONTROL2 = 0xA8,
} single_mode_cmd_e;

// 多电机控制模式ID定义，具体含义请参考电机协议文档
typedef enum {
	TORQUE_CONTROL = 0x280,
	SPEED_CONTROL = 0x281,
	POSITION_CONTROL = 0x282,
	MIX_CONTROL = 0x288,
} multi_mode_cmd_e;

// 电机控制参数命令
typedef enum {
    ANGLE_PID_PARAM     = 0x0A,
    SPEED_PID_PARAM     = 0x0B,
    CURRENT_PID_PARAM   = 0x0C,
    TORQUE_LIMIT_PARAM  = 0x1E,
    SPEED_LIMIT_PARAM   = 0x20,
    ANGLE_LIMIT_PARAM   = 0x22,
    CURRENT_RAMP_PARAM  = 0x24,
    SPEED_RAMP_PARAM    = 0x26,
}control_param_e;

/*
 * motor-id: DTS string -> const char*
 * control-mode: DTS enum -> DT_ENUM_IDX
 */
typedef struct motor_lk_cfg {
	uint16_t tx_id;
	uint16_t rx_id;
	int8_t motor_type;
	const char *motor_label;
	int8_t control_mode;
	uint16_t motor_encoder;
	uint8_t transmission_ratio;
	const struct device *can_dev;
#if defined(CONFIG_CAN_RX_MANAGER)
	const struct device *rx_mgr; // 可选：接收管理器
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
	const struct device *tx_mgr; // 可选：发送管理器
#endif
} motor_lk_cfg_t;

typedef struct motor_lk_data {
	smotor_data_t motor_data;
	motor_lk_single_data_t *single_data;        // 单电机专用数据，仅在单电机控制模式下使用
    uint16_t Tx_feq;                            // 发送频率，单位Hz，0表示仅手动发送
    struct k_spinlock lock;                     // 保护 motor_data 的自旋锁，防止接收更新和心跳检测冲突
    bool registered;
#if defined(CONFIG_CAN_RX_MANAGER)
	int rxmanager_slot_id; // CAN RX管理器 槽位ID
#endif

#if defined(CONFIG_MOTOR_HEARTBEAT_AUTOCHECK)
	const struct device *dev_self;   // 指向自身设备的指针，用于心跳自动检测
	struct k_work_delayable hb_work; // 心跳自动检测的延时工作
#endif
} motor_lk_data_t;


/*---------------------------------------------------lk_protocol----------------------------------------------------- */
/**
 * @brief 多电机控制模式下的帧解析函数，解析失败会返回负值，成功会返回0并更新data中的rx_data
 *
 * @param data	电机设备的接收结构体指针
 * @param frame	接收到的CAN帧指针
 * @return int
 */
static inline int motor_lk_multicontrol_iq(motor_lk_data_t *data, const struct can_frame *frame)
{
	if(data == NULL || frame == NULL) {
		LOG_ERR("[lk_motor_err] multicontrol Invalid arguments");
		return -EINVAL;
	}

	int cmd_id = frame->data[0];
	if (cmd_id != CLOSED_LOOP_CONTROL) {
		LOG_ERR("[lk_motor_err] cmd_id not match");
		return -EINVAL;
	}

	data->motor_data.rx_data.specific_data.lk.temp = (int8_t)frame->data[1];
	data->motor_data.rx_data.iq = (int16_t)(frame->data[3] << 8 | frame->data[2]);
	data->motor_data.rx_data.speed = (int16_t)(frame->data[5] << 8 | frame->data[4]);
	data->motor_data.rx_data.encoder = (uint16_t)(frame->data[7] << 8 | frame->data[6]);
	data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
							 						 MOTOR_RX_VALID_SPEED |
													 MOTOR_RX_VALID_IQ |
			   										 MOTOR_LK);
	return 0;
}

/**
 * @brief 单电机控制模式读取电机状态1，可以获取电机的母线电压，电流以及电机错误状态
 * 		  解析失败会返回负值，成功会返回0并更新data中的rx_data
 *
 * @param data	电机设备的接收结构体指针
 * @param frame	接收到的CAN帧指针
 * @return int
 */
static inline int motor_lk_readstate1(motor_lk_data_t *data, const struct can_frame *frame)
{
	if(data == NULL || frame == NULL) {
		LOG_ERR("[lk_motor_err] readstate Invalid arguments");
		return -EINVAL;
	}

	int cmd_id = frame->data[0];
	if (cmd_id != READ_MOTOR_STATE1 && cmd_id != CLEAR_MOTOR_ERROR) {
		LOG_ERR("[lk_motor_err] cmd_id not match");
		return -EINVAL;
	}

	data->motor_data.rx_data.specific_data.lk.temp = (int8_t)(frame->data[1]);
	data->motor_data.rx_data.specific_data.lk.vol = (int16_t)(frame->data[3] << 8 | frame->data[2]);
	data->motor_data.rx_data.specific_data.lk.current = (int16_t)(frame->data[5] << 8 | frame->data[4]);
	data->motor_data.rx_data.specific_data.lk.motorState = (uint8_t)(frame->data[6]);
	data->motor_data.rx_data.specific_data.lk.errorState = (uint8_t)(frame->data[7]);
	data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_LK);

	return 0;
}

/**
 * @brief 单电机控制模式读取电机状态2，可以获取电机温度，速度，编码器值
 * 		  解析失败会返回负值，成功会返回0并更新data中的rx_data
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readstate2(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
	if(data == NULL || frame == NULL || cfg == NULL) {
		LOG_ERR("[lk_motor_err] readstate2 Invalid arguments");
		return -EINVAL;
	}
	int cmd_id = frame->data[0];
	if (cmd_id != READ_MOTOR_STATE2 && cmd_id != OPEN_LOOP_CONTROL
        && cmd_id != CLOSED_LOOP_CONTROL && cmd_id != CLOSED_SPEED_CONTROL
        && cmd_id != MULTI_POSITION_CONTROL1 && cmd_id != MULTI_POSITION_CONTROL2
        && cmd_id != SINGLE_POSITION_CONTROL1 && cmd_id != SINGLE_POSITION_CONTROL2
        && cmd_id != INCRE_POSITION_CONTROL1 && cmd_id != INCRE_POSITION_CONTROL2) {
		LOG_ERR("[lk_motor_err] cmd_id not match");
		return -EINVAL;
	}

	data->motor_data.rx_data.specific_data.lk.temp = (int8_t)(frame->data[1]);
	data->motor_data.rx_data.speed = (int16_t)(frame->data[5] << 8 | frame->data[4]);
	data->motor_data.rx_data.encoder = (uint16_t)(frame->data[7] << 8 | frame->data[6]);
	if (cfg->motor_type == 3) // MS电机输出功率,MG,MF电机输出扭矩
	{
		data->motor_data.rx_data.specific_data.lk.power = (int16_t)(frame->data[3] << 8 | frame->data[2]);
		data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
										 				 MOTOR_RX_VALID_SPEED |
										 				 MOTOR_LK);
	}
	else
	{
		data->motor_data.rx_data.iq = (int16_t)(frame->data[3] << 8 | frame->data[2]);
		data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_RX_VALID_ENCODER |
										 				 MOTOR_RX_VALID_SPEED |
										 				 MOTOR_RX_VALID_IQ |
										 				 MOTOR_LK);
	}
	return 0;
}

/**
 * @brief 单电机控制模式读取电机状态3，可以获取电机各相电流
 * 		  解析失败会返回负值，成功会返回0并更新data中的rx_data
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readstate3(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
	if(data == NULL || frame == NULL || cfg == NULL) {
		LOG_ERR("[lk_motor_err] readstate3 Invalid arguments");
		return -EINVAL;
	}

	// MS电机不支持READ_MOTOR_STATE3命令
	if(cfg->motor_type == 3)
	{
		LOG_ERR("[lk_motor_err] MS motor does not support readstate3");
		return -EINVAL;
	}

	int cmd_id = frame->data[0];
	if(cmd_id != READ_MOTOR_STATE3)
	{
		LOG_ERR("[lk_motor_err] cmd_id not match");
		return -EINVAL;
	}

	data->motor_data.rx_data.specific_data.lk.temp = (int8_t)(frame->data[1]);
	data->motor_data.rx_data.specific_data.lk.iA = (int16_t)(frame->data[3] << 8 | frame->data[2]);
	data->motor_data.rx_data.specific_data.lk.iB = (int16_t)(frame->data[5] << 8 | frame->data[4]);
	data->motor_data.rx_data.specific_data.lk.iC = (int16_t)(frame->data[7] << 8 | frame->data[6]);
	data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_LK);
	return 0;

}


/**
 * @brief 单电机控制模式读取刹车状态，可以获取抱闸器状态
 *        解析失败会返回负值，成功会返回0并更新data中的rx_data
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readbreak(motor_lk_data_t *data, const struct can_frame *frame)
{
    if(data == NULL || frame == NULL) {
        LOG_ERR("[lk_motor_err] readbreak Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if(cmd_id != READ_BRAKE_STATE)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }
    // 0 表示刹车启动，1表示刹车释放
    data->motor_data.rx_data.specific_data.lk.holdBrakeState = (uint8_t)(frame->data[1]);
    data->motor_data.rx_data.valid_mask = (uint32_t)(MOTOR_LK);
    return 0;
}

/**
 * @brief 单电机控制模式读取控制参数，控制参数需要参考文档
 *        解析失败会返回负值，成功会返回0并更新data中的control_param
 *
 * @param data
 * @param frame
 * @return int
 */
static inline int motor_lk_readparam(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
    if(data == NULL || frame == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] readparam Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if(cmd_id != READ_CONTROL_PARAM)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }
    // 多电机控制下是用不到这个命令的，因此也不会分配内存
    if (cfg->control_mode == 0)
    {
        LOG_ERR("[lk_motor_err] multicontrol mode does not support readparam");
	    return -EINVAL;
    }

    data->single_data->control_param.controlParamID = (uint8_t)(frame->data[1]);
    switch(data->single_data->control_param.controlParamID)
    {
        case ANGLE_PID_PARAM:
            data->single_data->control_param.anglePidKp = (int16_t)(frame->data[3] << 8 | frame->data[2]);
            data->single_data->control_param.anglePidKi = (int16_t)(frame->data[5] << 8 | frame->data[4]);
            data->single_data->control_param.anglePidKd = (int16_t)(frame->data[7] << 8 | frame->data[6]);
            break;
        case SPEED_PID_PARAM:
            data->single_data->control_param.speedPidKp = (int16_t)(frame->data[3] << 8 | frame->data[2]);
            data->single_data->control_param.speedPidKi = (int16_t)(frame->data[5] << 8 | frame->data[4]);
            data->single_data->control_param.speedPidKd = (int16_t)(frame->data[7] << 8 | frame->data[6]);
            break;
        case CURRENT_PID_PARAM:
            data->single_data->control_param.currentPidKp = (int16_t)(frame->data[3] << 8 | frame->data[2]);
            data->single_data->control_param.currentPidKi = (int16_t)(frame->data[5] << 8 | frame->data[4]);
            data->single_data->control_param.currentPidKd = (int16_t)(frame->data[7] << 8 | frame->data[6]);
            break;
        case TORQUE_LIMIT_PARAM:
            data->single_data->control_param.torqueLimit = (int16_t)(frame->data[5] << 8 | frame->data[4]);
            break;
        case SPEED_LIMIT_PARAM:
            data->single_data->control_param.speedLimit = (int32_t)(frame->data[5] << 8 | frame->data[4] |
                                                                    frame->data[7] << 24 | frame->data[6] << 16);
            break;
        case ANGLE_LIMIT_PARAM:
            data->single_data->control_param.angleLimit = (int32_t)(frame->data[7] << 24 |
                                         frame->data[6] << 16 |
                                         frame->data[5] << 8 |
                                         frame->data[4]);
            break;
        case CURRENT_RAMP_PARAM:
            data->single_data->control_param.currentRamp = (int32_t)(frame->data[7] << 24 |
                                          frame->data[6] << 16 |
                                          frame->data[5] << 8 |
                                          frame->data[4]);
            break;
        case SPEED_RAMP_PARAM:
            data->single_data->control_param.speedRamp = (int32_t)(frame->data[7] << 24 |
                                        frame->data[6] << 16 |
                                        frame->data[5] << 8 |
                                        frame->data[4]);
            break;
        default:
            LOG_ERR("[lk_motor_err] unknown controlParamID: %d", data->single_data->control_param.controlParamID);
            return -EINVAL;
    }
    return 0;
}

/**
 * @brief 读取电机的编码器数据，包括原始位置，偏置。
 *        解析失败会返回负值，成功会返回0并更新data中的encoder_data
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readencoder(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
    if(data == NULL || frame == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] readencoder Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if(cmd_id != READ_ENCODER)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }
    if (cfg->control_mode == 0) {
	    LOG_ERR("[lk_motor_err] multicontrol mode does not support readencoder");
	    return -EINVAL;
    }

	data->single_data->encoder_data.encoder = (uint16_t)(frame->data[3] << 8 | frame->data[2]);
    data->single_data->encoder_data.encoderRaw = (uint16_t)(frame->data[5] << 8 | frame->data[4]);
    data->single_data->encoder_data.encoderOffset = (uint16_t)(frame->data[7] << 8 | frame->data[6]);
    return 0;
}

/**
 * @brief 单电机控制模式下读取多圈角度值，单位是0.01°/LSB ，支持正负值，正值表示顺时针旋转，负值表示逆时针旋转
 *       解析失败会返回负值，成功会返回0并更新data中的motorAngle
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readmultiencoder(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
    if (data == NULL || frame == NULL || cfg == NULL)
    {
        LOG_ERR("[lk_motor_err] readmultiencoder Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if (cmd_id != READ_MULTI_ENCODER)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }
    if (cfg->control_mode == 0) {
        LOG_ERR("[lk_motor_err] multicontrol mode does not support readmultiencoder");
        return -EINVAL;
    }

    data->single_data->motorAngle =
        ((int64_t)((uint64_t)frame->data[7] << 56) |
         (int64_t)((uint64_t)frame->data[6] << 48) |
         (int64_t)((uint64_t)frame->data[5] << 40) |
         (int64_t)((uint64_t)frame->data[4] << 32) |
         (int64_t)((uint64_t)frame->data[3] << 24) |
         (int64_t)((uint64_t)frame->data[2] << 16) |
         (int64_t)((uint64_t)frame->data[1] << 8)  |
         (int64_t)((uint64_t)frame->data[0]));
    return 0;

}

/**
 * @brief 单电机控制模式下读取单圈角度值，单位是0.01°/LSB，数值范围0~36000*减速比-1，以编码器零点为起始点，
 *         顺时针旋转增加，逆时针旋转减少，再次回到零点时，数值为0.
 *
 * @param data
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_readsingleencoder(motor_lk_data_t *data, const struct can_frame *frame, const motor_lk_cfg_t *cfg)
{
    if (data == NULL || frame == NULL || cfg == NULL)
    {
        LOG_ERR("[lk_motor_err] readsingleencoder Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if (cmd_id != READ_SINGLE_ENCODER)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }
    if (cfg->control_mode == 0) {
        LOG_ERR("[lk_motor_err] multicontrol mode does not support readsingleencoder");
        return -EINVAL;
    }

    data->single_data->circleAngle = (uint32_t)(frame->data[7] << 24 | frame->data[6] << 16 |
                                                frame->data[5] << 8 | frame->data[4]);
    return 0;
}

/**
 * @brief 单电机下设置编码器零点，电机收到该命令后会将当前位置的编码器值作为新的零点，写入ROM
 *        电机会反馈encoderOffset的值，注意多次写入会影响芯片的寿命
 *
 * @param data
 * @param frame
 * @return int
 */
static inline int motor_lk_setencoderzero(motor_lk_data_t *data, const struct can_frame *frame)
{
    if (data == NULL || frame == NULL)
    {
        LOG_ERR("[lk_motor_err] setencoderzero Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if (cmd_id != SET_ENCODER_ZERO)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }

    data->single_data->encoder_data.encoderOffset = (uint16_t)(frame->data[7] << 8 | frame->data[6]);
    return 0;
}

/**
 * @brief 单电机控制下，将当前位置设置为任意角度
 * @param data
 * @param frame
 * @return int
 */
static inline int motor_lk_setencoderself(motor_lk_data_t *data, const struct can_frame *frame)
{
    if (data == NULL || frame == NULL)
    {
        LOG_ERR("[lk_motor_err] setencoderself Invalid arguments");
        return -EINVAL;
    }

    int cmd_id = frame->data[0];
    if (cmd_id != SET_ENCODER_SELF)
    {
        LOG_ERR("[lk_motor_err] cmd_id not match");
        return -EINVAL;
    }

    // 驱动收到的和发送一致，这里就不需要做解析了。
    return 0;
}



/*---------------------------------------------------------lk motor single mode control api----------------------------------------------------------------------*/
/**
 * @brief 写入电机控制参数进入RAM，注意！掉电丢失！
 *
 * @param angleKp
 * @param angleKi
 * @param angleKd
 * @param frame
 * @return int
 */
static inline int motor_lk_writeparam_anglepid(const struct device *dev, int16_t angleKp, int16_t angleKi, int16_t angleKd)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] anglepid Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = ANGLE_PID_PARAM;
    data->motor_data.tx_data[2] = (uint8_t)(angleKp & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((angleKp >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(angleKi & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleKi >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)(angleKd & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleKd >> 8) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_speedpid(const struct device *dev, int16_t speedKp, int16_t speedKi, int16_t speedKd)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] speedpid Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = SPEED_PID_PARAM;
    data->motor_data.tx_data[2] = (uint8_t)(speedKp & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((speedKp >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(speedKi & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((speedKi >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)(speedKd & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((speedKd >> 8) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_currentpid(const struct device *dev, int16_t currentKp, int16_t currentKi, int16_t currentKd)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] currentpid Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = CURRENT_PID_PARAM;
    data->motor_data.tx_data[2] = (uint8_t)(currentKp & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((currentKp >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(currentKi & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((currentKi >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)(currentKd & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((currentKd >> 8) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_torquelimit(const struct device *dev, int16_t torqueLimit)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] torquelimit Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = TORQUE_LIMIT_PARAM;
    data->motor_data.tx_data[4] = (uint8_t)(torqueLimit & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((torqueLimit >> 8) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_speedlimit(const struct device *dev, int32_t speedLimit)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] speedlimit Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = SPEED_LIMIT_PARAM;
    data->motor_data.tx_data[4] = (uint8_t)(speedLimit & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((speedLimit >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((speedLimit >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((speedLimit >> 24) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_anglelimit(const struct device *dev, int32_t angleLimit)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] anglelimit Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = ANGLE_LIMIT_PARAM;
    data->motor_data.tx_data[4] = (uint8_t)(angleLimit & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleLimit >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleLimit >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleLimit >> 24) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_currentramp(const struct device *dev, int32_t currentRamp)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] currentramp Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = CURRENT_RAMP_PARAM;
    data->motor_data.tx_data[4] = (uint8_t)(currentRamp & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((currentRamp >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((currentRamp >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((currentRamp >> 24) & 0xFF);
    return 0;
}

static inline int motor_lk_writeparam_speedramp(const struct device *dev,int32_t speedRamp)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] speedramp Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = WRITE_CONTROL_PARAM;
    data->motor_data.tx_data[1] = SPEED_RAMP_PARAM;
    data->motor_data.tx_data[4] = (uint8_t)(speedRamp & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((speedRamp >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((speedRamp >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((speedRamp >> 24) & 0xFF);
    return 0;
}

/**
 * @brief 清除电机错误状态，返回与读取电机状态1一致
 *
 * @param frame
 * @return int
 */
static inline int motor_lk_clearerror(const struct device *dev)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] clearerror Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = CLEAR_MOTOR_ERROR;
    return 0;
}

/**
 * @brief 失能电机，清除之前的全部控制命令，LED由常亮变慢闪，此时电机还可以回复控制命令，但不会执行,反馈和主机相同
 *
 * @param frame
 * @return int
 */
static inline int motor_lk_disable(const struct device *dev)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] disable Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = DISABLE_MOTOR;
    return 0;
}

/**
 * @brief 使能电机，LED由慢闪变常亮，电机恢复执行控制命令，反馈和主机相同
 *
 * @param frame
 * @return int
 */
static inline int motor_lk_enable(const struct device *dev)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] enable Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = ENABLE_MOTOR;
    return 0;
}

/**
 * @brief 停止电机，但是不清除电机运行状态，再次发送控制命令可以控制电机，反馈和主机相同(1帧)
 *
 * @param frame
 * @return int
 */
static inline int motor_lk_stop(const struct device *dev)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] stop Invalid arguments");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = STOP_MOTOR;
    return 0;
}

/**
 * @brief 开环控制，仅在MS上可用，TODO: 入参应该是扭矩会好一点
 *
 * @param powerControl
 * @param frame
 * @param cfg
 * @return int
 */
static inline int motor_lk_single_openloopcontrol(const struct device *dev, int16_t powerControl)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] openloopcontrol Invalid arguments");
        return -EINVAL;
    }
     const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    if(cfg == NULL) {
        LOG_ERR("[lk_motor_err] openloopcontrol cfg is NULL");
        return -EINVAL;
    }
    if (cfg->motor_type != 3)
    {
        LOG_ERR("[lk_motor_err] openloopcontrol only support MS motor");
        return -EINVAL;
    }
    clamp(powerControl, -850, 850); // 电机的电流和扭矩因电机而异
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = OPEN_LOOP_CONTROL;
    data->motor_data.tx_data[4] = (uint8_t)(powerControl & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((powerControl >> 8) & 0xFF);
    return 0;
}

/**
 * @brief 单电机的转矩闭环控制命令，仅在MF，MH，MG上可用，数值范围是-2048~2048
 *        对应的MF电机实际转矩电流是-16.5A~16.5A，MG电机实际转矩电流是-33A~33A
 *
 * @param iqcontrol
 * @param frame
 * @return int
 */
static inline int motor_lk_single_closedloopcontrol(const struct device *dev, int16_t iqcontrol)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] closedloopcontrol Invalid arguments");
        return -EINVAL;
    }
    const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    if(cfg == NULL) {
        LOG_ERR("[lk_motor_err] closedloopcontrol cfg is NULL");
        return -EINVAL;
    }
     if (cfg->motor_type == 3)
    {
        LOG_ERR("[lk_motor_err] closedloopcontrol only support MG/MF/MH motor");
        return -EINVAL;
    }
    clamp(iqcontrol, -2048, 2048); // 电机的电流和扭矩因电机而异
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = CLOSED_LOOP_CONTROL;
    data->motor_data.tx_data[4] = (uint8_t)(iqcontrol & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((iqcontrol >> 8) & 0xFF);
    return 0;
}

/**
 * @brief 速度闭环控制，带有力矩限制，speedControl受上位机限制，反馈和读取电机状态2一致(仅命令字节data[0]不同)
 *        分辨率为0.01dps/LSB
 *
 * @param speedControl
 * @param iqcontrol
 * @param frame
 * @return int
 */
static inline int motor_lk_single_speedcontrol(const struct device *dev, double TargetspeedControl, int16_t iqcontrol)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    if(data == NULL) {
        LOG_ERR("[lk_motor_err] speedcontrol Invalid arguments");
        return -EINVAL;
    }
     const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    if(cfg == NULL) {
        LOG_ERR("[lk_motor_err] speedcontrol cfg is NULL");
        return -EINVAL;
    }
     if (cfg->motor_type == 3)
    {
        LOG_ERR("[lk_motor_err] speedcontrol only support MG/MF/MH motor");
        return -EINVAL;
    }
	clamp(iqcontrol, -2048, 2048); // 电机的速度控制范围因电机而异

    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t speedControl = (int32_t)(100 * TargetspeedControl * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的速度值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
	data->motor_data.tx_data[0] = CLOSED_SPEED_CONTROL;
    data->motor_data.tx_data[2] = (uint8_t)(iqcontrol & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((iqcontrol >> 8) & 0xFF);
	data->motor_data.tx_data[4] = (uint8_t)(speedControl & 0xFF);
	data->motor_data.tx_data[5] = (uint8_t)((speedControl >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((speedControl >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((speedControl >> 24) & 0xFF);
	return 0;
}

/* 请注意！官方说的0.01degree/LSB是指电机转子，但是实际的输出轴还需要乘减速比，
*  本API将乘上减速比，如果减速比在设备树配置的不对，计算就会有误！！！！例如：希望目标输出轴转360°，那么入参直接写360即可
*/

/**
 * @brief 多圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°
 *        受上位机的速度，位置等最大值限制。反馈和读取电机状态2一致(仅命令字节data[0]不同)
 * @param angleControl
 * @param frame
 * @return int
 */
static inline int motor_lk_single_mulposcontrol1(const struct device *dev, double Targetangle)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] mulposcontrol1 Invalid arguments");
        return -EINVAL;
    }

    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t angleControl = (int32_t)(100 * Targetangle * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的角度值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = MULTI_POSITION_CONTROL1;
    data->motor_data.tx_data[4] = (uint8_t)(angleControl & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleControl >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleControl >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleControl >> 24) & 0xFF);
    return 0;
}

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
static inline int motor_lk_single_mulposcontrol2(const struct device *dev, double Targetangle, double TargetmaxSpeed)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] mulposcontrol2 Invalid arguments");
        return -EINVAL;
    }
    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t angleControl = (int32_t)(100 * Targetangle * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的角度值
    uint16_t maxSpeed = (uint16_t)(trans_ratio * TargetmaxSpeed); // 转速也要乘上减速比，得到电机输出轴的转速值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = MULTI_POSITION_CONTROL2;
    data->motor_data.tx_data[2] = (uint8_t)(maxSpeed & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((maxSpeed >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(angleControl & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleControl >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleControl >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleControl >> 24) & 0xFF);
    return 0;
}

/**
 * @brief 单圈位置闭环控制命令1，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数。
 *
 * @param spindir
 * @param angleControl
 * @param frame
 * @return int
 */
static inline int motor_lk_single_sigposcontrol1(const struct device *dev, bool spindir, double Targetangle)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] sigposcontrol1 Invalid arguments");
        return -EINVAL;
    }

    uint8_t trans_ratio = cfg->transmission_ratio;
    uint32_t angleControl = (uint32_t)(100 * Targetangle * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的角度值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = SINGLE_POSITION_CONTROL1;
    data->motor_data.tx_data[1] = spindir ? 1 : 0;                           // 0表示顺时针旋转，1表示逆时针旋转
    data->motor_data.tx_data[4] = (uint8_t)(angleControl & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleControl >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleControl >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleControl >> 24) & 0xFF);
    return 0;
}

/**
 * @brief 单圈位置闭环控制命令2，对应实际位置为0.01degree/LSB,也就是36000代表360°，带有转向参数和速度限制。
 *
 * @param spindir
 * @param angleControl
 * @param maxSpeed
 * @param frame
 * @return int
 */
static inline int motor_lk_single_sigposcontrol2(const struct device *dev, bool spindir, double Targetangle, double TargetmaxSpeed)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] sigposcontrol2 Invalid arguments");
        return -EINVAL;
    }
    uint8_t trans_ratio = cfg->transmission_ratio;
    uint32_t angleControl = (uint32_t)(100 * Targetangle * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的角度值
    uint16_t maxSpeed = (uint16_t)(trans_ratio * TargetmaxSpeed); // 转速也要乘上减速比，得到电机输出轴的转速值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = SINGLE_POSITION_CONTROL2;
    data->motor_data.tx_data[1] = spindir ? 1 : 0;                           // 0表示顺时针旋转，1表示逆时针旋转
    data->motor_data.tx_data[2] = (uint8_t)(maxSpeed & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((maxSpeed >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(angleControl & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleControl >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleControl >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleControl >> 24) & 0xFF);
    return 0;
}

/**
 * @brief 增量位置闭环控制命令1，对应实际增量位置为0.01degree/LSB,也就是36000代表360°
 *
 * @param angleIncre
 * @param frame
 * @return int
 */
static inline int motor_lk_single_increposcontrol1(const struct device *dev, double TargetangleIncre)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] increposcontrol1 Invalid arguments");
        return -EINVAL;
    }

    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t angleIncre = (int32_t)(100 * TargetangleIncre * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的增量角度值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = INCRE_POSITION_CONTROL1;
    data->motor_data.tx_data[4] = (uint8_t)(angleIncre & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleIncre >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleIncre >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleIncre >> 24) & 0xFF);
    return 0;
}

/**
 * @brief 增量位置闭环控制命令2，对应实际增量位置为0.01degree/LSB,也就是36000代表360°，带有速度限制
 *
 * @param angleIncre
 * @param maxSpeed
 * @param frame
 * @return int
 */
static inline int motor_lk_single_increposcontrol2(const struct device *dev,double TargetangleIncre, double TargetmaxSpeed)
{
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;
    motor_lk_cfg_t *cfg = (motor_lk_cfg_t *)dev->config;
    if(data == NULL || cfg == NULL) {
        LOG_ERR("[lk_motor_err] increposcontrol2 Invalid arguments");
        return -EINVAL;
    }
    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t angleIncre = (int32_t)(100 * TargetangleIncre * trans_ratio); // 乘上减速比，得到电机输出轴需要达到的增量角度值
    uint32_t maxSpeed = (uint32_t)(trans_ratio * TargetmaxSpeed); // 转速也要乘上减速比，得到电机输出轴的转速值

    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = INCRE_POSITION_CONTROL2;
    data->motor_data.tx_data[2] = (uint8_t)(maxSpeed & 0xFF);
    data->motor_data.tx_data[3] = (uint8_t)((maxSpeed >> 8) & 0xFF);
    data->motor_data.tx_data[4] = (uint8_t)(angleIncre & 0xFF);
    data->motor_data.tx_data[5] = (uint8_t)((angleIncre >> 8) & 0xFF);
    data->motor_data.tx_data[6] = (uint8_t)((angleIncre >> 16) & 0xFF);
    data->motor_data.tx_data[7] = (uint8_t)((angleIncre >> 24) & 0xFF);
    return 0;
}

/*---------------------------------------------------------lk motor multi mode control api----------------------------------------------------------------------*/

/**
 * @brief 多电机扭矩控制.
 * 
 * @param dev 
 * @param current 
 * @return int 
 */
static inline int motor_lk_multi_torquecontrol(const struct device *dev, int16_t current)
{
    const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;

    switch(cfg->motor_type)
    {
        case 1: // MG电机
        case 2: // MF电机
            clamp(current, -MGMF_CURRENT_MAX, MGMF_CURRENT_MAX); // 电机的电流和扭矩因电机而异
            break;
        case 3: // MS电机
            clamp(current, -MS_CURRENT_MAX, MS_CURRENT_MAX); // MS电机是电压控制
            return -EINVAL;
        case 4: // MH电机               // MH电机的限制值未知
            break;
        default:
            LOG_ERR("[lk_motor_err] unknown motor type: %d", cfg->motor_type);
            return -EINVAL;
    }
    if(cfg->tx_id != 0x280)
    {
        LOG_ERR("[lk_motor_err] torque control only support tx_id 0x280");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = (uint8_t)(current & 0xFF);
    data->motor_data.tx_data[1] = (uint8_t)((current >> 8) & 0xFF);

    return 0;
}

/**
 * @brief 多电机的速度控制命令，speedValue的范围是-32768 ~ 32767dps，分辨率是1dps/LSB
 * 
 * @param dev 
 * @param speedValue 
 * @return int 
 */
static inline int motor_lk_multi_speedcontrol(const struct device *dev, double TargetspeedValue)
{
    const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;

    if(cfg->tx_id != 0x281)
    {
        LOG_ERR("[lk_motor_err] speed control only support tx_id 0x281");
        return -EINVAL;
    }

    uint8_t trans_ratio = cfg->transmission_ratio;

    int16_t speedValue = (int16_t)(TargetspeedValue * trans_ratio);
    clamp(speedValue, -SPEEDVALUE_MAX, SPEEDVALUE_MAX-1); // 电机的速度控制范围因电机而异
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = (uint8_t)(speedValue & 0xFF);
    data->motor_data.tx_data[1] = (uint8_t)((speedValue >> 8) & 0xFF);
    return 0;
}

/**
 * @brief 多电机的位置控制命令，angleValue的范围是-32768 ~ 32767dps，分辨率是0.01degree/LSB，也就是36000代表360°，
 * 
 * @param dev 
 * @param angleValue 
 * @return int 
 */
static inline int motor_lk_multi_positcontrol(const struct device *dev, double TargetangleValue)
{
    const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;

    if(cfg->tx_id != 0x282)
    {
        LOG_ERR("[lk_motor_err] position control only support tx_id 0x282");
        return -EINVAL;
    }
    uint8_t trans_ratio = cfg->transmission_ratio;
    int32_t angleValue = (int32_t)(TargetangleValue * trans_ratio);
    clamp(angleValue, -POSITVALUE_MAX, POSITVALUE_MAX-1); // 电机的位置控制范围因电机而异
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = (uint8_t)(angleValue & 0xFF);
    data->motor_data.tx_data[1] = (uint8_t)((angleValue >> 8) & 0xFF);
    return 0;
}

/**
 * @brief 多电机下的混合命令，motor_cmd就是单电机下的控制命令
          支持0x9A,0x9B,0x9C,0x80,0x88,0x81。其他暂不支持
 * 
 * @param dev 
 * @param motor_cmd 
 * @return int 
 */
static inline int motor_lk_multi_mixcontrol(const struct device *dev, uint16_t motor_cmd)
{
    const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)dev->config;
    motor_lk_data_t *data = (motor_lk_data_t *)dev->data;

    if(cfg->tx_id != 0x288)
    {
        LOG_ERR("[lk_motor_err] mix control only support tx_id 0x288");
        return -EINVAL;
    }
    memset(data->motor_data.tx_data, 0, 8); // 先清零，避免之前的控制命令对这次控制造成影响
    data->motor_data.tx_data[0] = (uint8_t)(motor_cmd & 0xFF);
    data->motor_data.tx_data[1] = (uint8_t)(0x00);
    return 0;

}

#endif // LK_PROTOCOL_H
