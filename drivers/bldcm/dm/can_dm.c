/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_dm_can_motor

#include "dm_protocol.h"
#include <zephyr/sys/util_macro.h>
#include <errno.h>
#include <string.h>
#include <stddef.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/slist.h>

int motor_dm_update_heartbeat_status(const struct device *dev);
static int motor_dm_txbuff_init(const struct device *dev);

/**
 * @brief 心跳自动检测工作处理函数
 *
 */
#if defined(CONFIG_BLDCM_HEARTBEAT_AUTOCHECK)
static void motor_dm_hb_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork =
		k_work_delayable_from_work(work); // 获取 k_work_delayable 指针
	motor_dm_data_t *data =
		CONTAINER_OF(dwork, motor_dm_data_t, hb_work); // 获取 motor_dm_data_t 指针

	if (data->dev_self != NULL) {
		(void)motor_dm_update_heartbeat_status(data->dev_self);
		(void)k_work_schedule(
			&data->hb_work,
			K_MSEC(CONFIG_BLDCM_HEARTBEAT_POLL_PERIOD_MS)); // 重新调度下一次心跳检测
	}
}
#endif

#if defined(CONFIG_CAN_RX_MANAGER)
static void motor_dm_can_rx_handler(const struct can_frame *frame, void *user_data)
{
    struct device *motor_dev = (struct device *)user_data;
	if ((motor_dev == NULL) || (frame == NULL)) {
		LOG_ERR("[dm_motor_err] rx handle Invalid arguments");
		return;
	}

    motor_dm_data_t *data = (motor_dm_data_t *)motor_dev->data; // 获取 motor 数据结构体指针
	const motor_dm_cfg_t *cfg = (const motor_dm_cfg_t *)motor_dev->config; // 获取 motor 配置结构体指针
    int ret;
	if (cfg == NULL || data == NULL) {
		LOG_ERR("[dm_motor_err] data and cfg struct is NULL");
		return;
	}

	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		LOG_ERR("[dm_motor_err] RTR frame received");
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock); // 加锁保护 motor_data
	data->motor_data.heartbeat_status.is_alive = true;
    if(cfg->control_mode == 0)
    {
        LOG_WRN_RATELIMIT_RATE(1000, "[dm_motor_wrn]: boardcast mode is not supported yet, received frame with ID: 0x%X", frame->id);
        k_spin_unlock(&data->lock, key);
        return;
    }
    else if((frame->data[0] & 0x0F) == cfg->tx_id)
    {
        ret = motor_dm_rev_data(motor_dev, frame); // 达妙电机非多电机模式的反馈报文是一样的
        if (ret != 0) {
            LOG_ERR("[dm_motor_err] Failed to process received data: %d", ret);
        }
    }
    else
    {
        switch(frame->data[2])
        {
            case DM_CMD_R_REG:
                motor_dm_readreg_back(&data->reg_info,frame);
                break;
            case DM_CMD_W_REG:  // 不需要处理
                break;
            case DM_CMD_STORE:
                motor_dm_storereg_back(frame);      // 当前只有日志说明是否存储成功
				break;
            default:
                LOG_ERR("[dm_motor_err] Received frame with unrecognized command: 0x%02X", frame->data[2]);
                break;
        }
    }
    data->motor_data.heartbeat_status.heartbeat_tick = (uint64_t)k_uptime_get();
	k_spin_unlock(&data->lock, key);
}
#endif

#if defined(CONFIG_CAN_TX_MANAGER)
static int motor_dm_can_tx_fillbuffer_handler(struct can_frame *frame, void *user_data)
{
    struct device *motor_dev = (struct device *)user_data;
	if ((motor_dev == NULL) || (frame == NULL)) {
		LOG_ERR("[dm_motor_err] tx fillbuffer handle Invalid arguments");
		return -EINVAL;
	}

    motor_dm_data_t *data = (motor_dm_data_t *)motor_dev->data; // 获取 motor 数据结构体指针
	const motor_dm_cfg_t *cfg = (const motor_dm_cfg_t *)motor_dev->config; // 获取 motor 配置结构体指针
	if (cfg == NULL || data == NULL || frame == NULL) {
		LOG_ERR("[dm_motor_err] data and cfg struct is NULL");
		return -EINVAL;
	}

	 // 加锁保护 motor_data
	switch(cfg->control_mode)
	{
		case 0:             // 多电机控制模式, 只有多电机控制模式下才需要特殊处理
		{
			k_spinlock_key_t key = k_spin_lock(&data->lock);
			frame->dlc = 8;
			frame->flags = 0;
			int diff = cfg->rx_id & 0x0F;
			if (diff <= 0 || diff > 8) {
				LOG_ERR("[dm_motor_err] tx handle invalid id difference: tx_id=%d, rx_id=%d", cfg->tx_id, cfg->rx_id);
				k_spin_unlock(&data->lock, key);
				return -EINVAL;
			}
			int off = (diff > 4) ? (diff - 4) : diff;
			if (off <= 0 || off > 4) {
				LOG_ERR("[dm_motor_err] tx handle computed off out of range: %d", off);
				k_spin_unlock(&data->lock, key);
				return -EINVAL;
			}
			int idx = 2 * (off - 1);
			if ((idx + 2) > 8) {
				LOG_ERR("[dm_motor_err] tx handle target index overflow: idx=%d", idx);
				k_spin_unlock(&data->lock, key);
				return -EFAULT;
			}
			memcpy(&frame->data[idx], &data->motor_data.tx_data[0], 2);
			k_spin_unlock(&data->lock, key);
			return 0;
		}
		case 1:     // MIT
        case 2:     // pos_vel
		{	k_spinlock_key_t key = k_spin_lock(&data->lock);
			frame->dlc = 8;
			frame->flags = 0;
			memcpy(&frame->data[0], &data->motor_data.tx_data[0], 8);
			k_spin_unlock(&data->lock, key);
			return 0;
		}
        case 3:     // vel, dlc 应为 4
		{	k_spinlock_key_t key = k_spin_lock(&data->lock);
			frame->dlc = 4;
			frame->flags = 0;
			memcpy(&frame->data[0], &data->motor_data.tx_data[0], 4);
			k_spin_unlock(&data->lock, key);
			return 0;
		}
		default:
		{	LOG_ERR("[dm_motor_err] tx handle invalid control mode: %d", cfg->control_mode);
			return -EINVAL;
		}
	}
	return 0;
}
#endif

/**
 * @brief 单电机注册接口，通过 motor_id 识别电机实例，配置了can的接收过滤器
 *
 * @param dev
 * @return int
 */
static int motor_dm_can_register_motor(const struct device *dev)
{
	const motor_dm_cfg_t *cfg = dev->config;
	motor_dm_data_t *data = dev->data;

	if ((cfg == NULL) || (data == NULL)) {
		LOG_ERR("[dm_motor_err] register Invalid arguments");
		return -EINVAL;
	}

	if (!device_is_ready(cfg->can_dev)) {
		LOG_ERR("[dm_motor_err] CAN device not ready");
		return -ENODEV;
	}

	if (data->registered) {
		LOG_WRN("[dm_motor_err] motor already registered,please check your code");
		return -EALREADY;
	}
#if defined(CONFIG_CAN_RX_MANAGER)
	if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
		LOG_ERR("[dm_motor_err] RX manager not ready");
		return -ENODEV;
	}
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
	if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
		LOG_ERR("[dm_motor_err] TX manager not ready");
		return -ENODEV;
	}
#endif

	(void)motor_dm_txbuff_init(dev);

	// can 过滤器
	struct can_filter filter = {
		.id = cfg->rx_id & CAN_STD_ID_MASK,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	int rx_ret = -1;

#if defined(CONFIG_CAN_RX_MANAGER) // 将电机接收交给 CAN RX 管理器处理
	rx_ret = can_rx_manager_register(cfg->rx_mgr, &filter, motor_dm_can_rx_handler,
					 (void *)dev);
	if (rx_ret < 0) {
		LOG_ERR("[dm_motor_err] Failed to register motor on RxManager: %d", rx_ret);
		return rx_ret;
	} else {
		LOG_INF("Motor (%s) registered on RxManager, CAN RX ID: 0x%03X  slotID: %d",
			cfg->motor_label, cfg->rx_id, rx_ret);
	}
	data->rxmanager_slot_id = rx_ret;
#endif

#if defined(CONFIG_CAN_TX_MANAGER)
	int tx_ret = -1;
	tx_ret = can_tx_manager_register(cfg->tx_mgr, cfg->tx_id, cfg->rx_id, 8, 0, data->Tx_feq,
					 motor_dm_can_tx_fillbuffer_handler, (void *)dev);
	if (tx_ret < 0) {
		LOG_ERR("[dm_motor_err] Failed to register CAN TX filter: %d", tx_ret);
		return tx_ret;
	} else {
		LOG_INF("Motor (%s) registered on TxManager, CAN TX ID: 0x%03X",
			cfg->motor_label, cfg->tx_id);
	}
#else
	LOG_INF("Motor (%s) did not register on TxManager, CAN TX ID: 0x%03X", cfg->motor_label,
		cfg->tx_id);
#endif

	data->registered = true;
	data->motor_data.interface_ptr = (void *)cfg;
	data->motor_data.rx_data.valid_mask = 0U;
	data->motor_data.heartbeat_status.is_alive = false;
	data->motor_data.heartbeat_status.heartbeat_tick = 0;
	return 0;
}

int motor_dm_update_heartbeat_status(const struct device *dev)
{
	if (dev == NULL) {
		LOG_ERR("[dm_motor_err] update heartbeat Invalid arguments");
		return -EINVAL;
	}

	motor_dm_data_t *data = dev->data;
	if (data == NULL) {
		LOG_ERR("[dm_motor_err] update heartbeat data NULL");
		return -EINVAL;
	}
	uint64_t elapsed = 0;
	const motor_dm_cfg_t *cfg = (const motor_dm_cfg_t *)data->motor_data.interface_ptr;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint64_t last_tick = data->motor_data.heartbeat_status.heartbeat_tick;
	uint64_t current_tick = (uint64_t)k_uptime_get();
	bool prev_alive = data->motor_data.heartbeat_status.is_alive;

	/* 尚未收到过任何帧时，每隔 1s 告警一次，避免静默失败。 */
	if (last_tick == 0U) {
		LOG_WRN_RATELIMIT_RATE(1000,
			"[dm_motor_err] motor offline! no first CAN frame received (%s, rx=0x%03x)",
			(cfg != NULL && cfg->motor_label != NULL) ? cfg->motor_label : "unknown",
			(cfg != NULL) ? (unsigned int)cfg->rx_id : 0U);
		data->motor_data.heartbeat_status.is_alive = false;
		k_spin_unlock(&data->lock, key);
		return 0;
	}
	elapsed = current_tick - last_tick;
	/* 如果超过阈值没有收到心跳，则认为电机掉线：清零接收值并在离线边沿告警一次 */
	if (elapsed > (uint64_t)CONFIG_BLDCM_HEARTBEAT_OFFLINE_TIMEOUT_MS) {
		data->motor_data.heartbeat_status.is_alive = false;

		/* 只有从在线->离线时，才清零并告警；避免每次轮询刷屏 */
		if (prev_alive) {
			memset(&data->motor_data.rx_data, 0, sizeof(data->motor_data.rx_data));
			LOG_ERR("[dm_motor_err] motor offline (%s, rx=0x%03x): no CAN frames for "
				"%llu ms",
				(cfg != NULL && cfg->motor_label != NULL) ? cfg->motor_label
									  : "unknown",
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

static int motor_dm_can_change_tx_feq(const struct device *dev, uint16_t new_feq)
{
    motor_dm_data_t *data = dev->data;
    if (data == NULL) {
        LOG_ERR("[dm_motor_err] change Tx feq Invalid arguments");
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->Tx_feq = new_feq;
    k_spin_unlock(&data->lock, key);
    LOG_WRN("[dm_motor] Tx frequency changed to %u Hz", new_feq);
    return 0;
}

static int motor_dm_can_get_heartbeat_status(const struct device *dev)
{
    int ret = motor_dm_update_heartbeat_status(dev);
    if (ret < 0) {
        return ret;
    }

	motor_dm_data_t *data = dev->data;
    if (data == NULL) {
        return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    bool alive = data->motor_data.heartbeat_status.is_alive;
    k_spin_unlock(&data->lock, key);

    return alive ? 1 : 0;
}

/**
 * @brief 暴露给中间件获取电机数据的接口，Atention!!!!!:
 *        这里直接返回了 Rx_data 的指针，上层不可更改
 *        此外因为大部分mcu都是单核的，主线程和中断不会并发执行，所以是安全的
 *        如果在多核平台上使用，请自行加锁保护！！！！！或者改成双缓冲及快照
 *
 * @param dev
 * @return const smotor_receive_data_t*
 */
static const smotor_receive_data_t *motor_dm_can_get_rxdata(const struct device *dev)
{
    motor_dm_data_t *data = dev->data;
    if (data == NULL) {
        LOG_WRN("[dm_motor_err] get_rxdata dev NULL");
        return NULL;
    }

    return &data->motor_data.rx_data;
}

const dm_special_api_t motor_dm_can_special_api = {
    .dm_read_reg = motor_dm_read_register,
    .dm_write_reg = motor_dm_write_register,
    .dm_store_reg = motor_dm_store,
    .dm_save_zero = motor_dm_save_zero_pos,
    .dm_mit_control = motor_dm_mit_ctrl,
    .dm_posvel_control = motor_dm_posvel_ctrl,
    .dm_vel_control = motor_dm_vel_ctrl,
};


const motor_driver_api_t motor_dm_can_api = {
	.register_motor = motor_dm_can_register_motor,
    .get_heartbeat_status = motor_dm_can_get_heartbeat_status,
	.torque_control = NULL,
	.change_tx_feq = motor_dm_can_change_tx_feq,
    .get_rxdata = motor_dm_can_get_rxdata,
	.clear_error = motor_dm_clear_error,
	.disable = motor_dm_disable,
	.enable = motor_dm_enable,
	.stop = NULL,
    .dm_api = &motor_dm_can_special_api,
};

/**
 * @brief check if the control mdoe is MIT mode, if so, the txbuff need to be mapped
 *
 * @param dev
 * @return int
 */
static int motor_dm_txbuff_init(const struct device *dev)
{
	motor_dm_data_t *data = dev->data;
	const motor_dm_cfg_t *cfg = dev->config;
	if (data == NULL || cfg == NULL) {
		LOG_ERR("[dm_motor_err] txbuff init Invalid arguments");
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	if (cfg->control_mode == 1)		// MIT 模式下发送数组需要映射
	{
		data->motor_data.tx_data[0] = (float_to_uint(0, cfg->param_limit.pos_min, cfg->param_limit.pos_max, 16) >> 8) & 0xFF;
    	data->motor_data.tx_data[1] = float_to_uint(0, cfg->param_limit.pos_min, cfg->param_limit.pos_max, 16) & 0xFF;
    	data->motor_data.tx_data[2] = (float_to_uint(0, cfg->param_limit.vel_min, cfg->param_limit.vel_max, 12) >> 4) & 0xFF;
    	data->motor_data.tx_data[3] = ((float_to_uint(0, cfg->param_limit.vel_min, cfg->param_limit.vel_max, 12) & 0x0F) << 4)
                                	| ((float_to_uint(0, cfg->param_limit.kp_min, cfg->param_limit.kp_max, 12) >> 8) & 0x0F);
    	data->motor_data.tx_data[4] = float_to_uint(0, cfg->param_limit.kp_min, cfg->param_limit.kp_max, 12) & 0xFF;
    	data->motor_data.tx_data[5] = (float_to_uint(0, cfg->param_limit.kd_min, cfg->param_limit.kd_max, 12) >> 4) & 0xFF;
    	data->motor_data.tx_data[6] = ((float_to_uint(0, cfg->param_limit.kd_min, cfg->param_limit.kd_max, 12) & 0x0F) << 4)
                                	| ((float_to_uint(0, cfg->param_limit.tq_min, cfg->param_limit.tq_max, 12) >> 8) & 0x0F);
    	data->motor_data.tx_data[7] = float_to_uint(0, cfg->param_limit.tq_min, cfg->param_limit.tq_max, 12) & 0xFF;
	}
	else	// 非 MIT 模式清零发送缓冲
	{
		memset(data->motor_data.tx_data, 0, sizeof(data->motor_data.tx_data));
	}
	k_spin_unlock(&data->lock, key);
	return 0;
}

/**
 * @brief dm电机实例的初始化
 *
 * @param dev
 * @return int
 */
int motor_dm_can_init(const struct device *dev)
{
	const motor_dm_cfg_t *cfg = dev->config;
	motor_dm_data_t *data = dev->data;

	if (!device_is_ready(cfg->can_dev)) {
		return -ENODEV;
	}

	int start_ret = can_start(cfg->can_dev);
	if ((start_ret < 0) && (start_ret != -EALREADY)) {
		LOG_ERR("[dm_motor_err] Failed to start CAN device, error: %d", start_ret);
		return start_ret;
	}

#if defined(CONFIG_CAN_RX_MANAGER)
	if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
		__ASSERT(false, "[dm_motor_err] RX manager not ready for device %s", dev->name);
		return -ENODEV;
	}
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
	if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
		__ASSERT(false, "[dm_motor_err] TX manager not ready for device %s", dev->name);
		return -ENODEV;
	}
#endif

	data->registered = false;
	memset(&data->motor_data, 0, sizeof(data->motor_data));
	memset(&data->reg_info, 0, sizeof(data->reg_info));
	data->motor_data.interface_ptr = (void *)cfg;
	data->motor_data.rx_data.valid_mask = 0U;
	data->motor_data.heartbeat_status.is_alive = false;
	data->motor_data.heartbeat_status.heartbeat_tick = 0;
	start_ret = motor_dm_txbuff_init(dev);
	if (start_ret < 0) {
		LOG_ERR("[dm_motor_err] Failed to initialize tx buffer, error: %d", start_ret);
		return start_ret;
	}

#if defined(CONFIG_BLDCM_HEARTBEAT_AUTOCHECK)
	data->dev_self = dev;
	k_work_init_delayable(&data->hb_work, motor_dm_hb_work_handler);
	(void)k_work_schedule(&data->hb_work, K_MSEC(CONFIG_BLDCM_HEARTBEAT_POLL_PERIOD_MS));
#endif

	return 0;
}

/* ---------- Devicetree helpers ---------- */

/* 获取 control-mode enum 的索引；未配置则返回 -1 */
#define MOTOR_DM_CONTROL_MODE(inst)                                                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, control_mode), (DT_INST_ENUM_IDX(inst, control_mode)), (-1))

#define MOTOR_DM_TYPE(inst)                                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, motor_type), (DT_INST_ENUM_IDX(inst, motor_type)), (-1))

#define MOTOR_DM_LIMIT_SCALE 1000.0f

#define MOTOR_DM_DEFINE(inst)                                                                                                                                                      \
	static const motor_dm_cfg_t motor_dm_cfg_##inst = {                                      \
		.tx_id = (uint16_t)DT_INST_PROP(inst, tx_id),                                      \
		.rx_id = (uint16_t)DT_INST_PROP(inst, rx_id),                                      \
		.motor_label = DT_INST_PROP(inst, label),                                          \
		.motor_type = (int8_t)MOTOR_DM_TYPE(inst),                                        \
		.control_mode = (int8_t)MOTOR_DM_CONTROL_MODE(inst),                              \
		.motor_encoder = (uint16_t)DT_INST_PROP(inst, motor_encoder),                      \
		.transmission_ratio = (uint8_t)DT_INST_PROP(inst, motor_transmission_ratio),       \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                          \
		.param_limit = {                                                                    \
			.vel_max = (float)(int32_t)DT_INST_PROP(inst, vel_max) / MOTOR_DM_LIMIT_SCALE, \
			.vel_min = -(float)(int32_t)DT_INST_PROP(inst, vel_max) / MOTOR_DM_LIMIT_SCALE, \
			.tq_max = (float)(int32_t)DT_INST_PROP(inst, tq_max) / MOTOR_DM_LIMIT_SCALE,   \
			.tq_min = -(float)(int32_t)DT_INST_PROP(inst, tq_max) / MOTOR_DM_LIMIT_SCALE,   \
			.pos_max = (float)(int32_t)DT_INST_PROP(inst, pos_max) / MOTOR_DM_LIMIT_SCALE, \
			.pos_min = -(float)(int32_t)DT_INST_PROP(inst, pos_max) / MOTOR_DM_LIMIT_SCALE, \
			.kp_max = (float)(int32_t)DT_INST_PROP_BY_IDX(inst, mit_params_limit, 0) / MOTOR_DM_LIMIT_SCALE, \
			.kp_min = (float)(int32_t)DT_INST_PROP_BY_IDX(inst, mit_params_limit, 1) / MOTOR_DM_LIMIT_SCALE, \
			.kd_max = (float)(int32_t)DT_INST_PROP_BY_IDX(inst, mit_params_limit, 2) / MOTOR_DM_LIMIT_SCALE, \
			.kd_min = (float)(int32_t)DT_INST_PROP_BY_IDX(inst, mit_params_limit, 3) / MOTOR_DM_LIMIT_SCALE, \
        },                                                                                     \
		IF_ENABLED(CONFIG_CAN_RX_MANAGER, ( \
			.rx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rx_manager), \
				      (DEVICE_DT_GET(DT_INST_PHANDLE(inst, rx_manager))), (NULL)), \
		)) \
		IF_ENABLED(CONFIG_CAN_TX_MANAGER, ( \
			.tx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, tx_manager), \
			      (DEVICE_DT_GET(DT_INST_PHANDLE(inst, tx_manager))), (NULL)), \
		)) \
    };                                                                                         \
	static motor_dm_data_t motor_dm_data_##inst = {                                          \
		.Tx_feq = (uint16_t)DT_INST_PROP(inst, tx_feq),                                    \
	};                                                                                         \
    DEVICE_DT_INST_DEFINE(inst, motor_dm_can_init, NULL, &motor_dm_data_##inst, \
                      &motor_dm_cfg_##inst, POST_KERNEL, CONFIG_BLDCM_INIT_PRIORITY, \
                  &motor_dm_can_api);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(MOTOR_DM_DEFINE)
#endif
