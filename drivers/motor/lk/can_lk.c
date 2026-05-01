/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_lk_can_motor

#include "lk_protocol.h"
#include <zephyr/sys/util_macro.h>
#include <errno.h>
#include <string.h>
#include <stddef.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/slist.h>


int motor_lk_update_heartbeat_status(const struct device *dev);
int motor_lk_can_heartbeat_probe(const struct device *dev);

/**
 * @brief 心跳自动检测工作处理函数
 *
 */
#if defined(CONFIG_MOTOR_HEARTBEAT_AUTOCHECK)
static void motor_lk_hb_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork =
		k_work_delayable_from_work(work); // 获取 k_work_delayable 指针
	motor_lk_data_t *data =
		CONTAINER_OF(dwork, motor_lk_data_t, hb_work); // 获取 motor_lk_data_t 指针

	if (data->dev_self != NULL) {
		// (void)motor_lk_can_heartbeat_probe(data->dev_self);
		(void)motor_lk_update_heartbeat_status(data->dev_self);
		(void)k_work_schedule(
			&data->hb_work,
			K_MSEC(CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS)); // 重新调度下一次心跳检测
	}
}
#endif

#if defined(CONFIG_CAN_RX_MANAGER)
// 修正回调类型，参数加const
static void motor_lk_can_rx_handler(const struct can_frame *frame, void *user_data)
{
	const struct device *motor_dev = (const struct device *)user_data;
	if ((motor_dev == NULL) || (frame == NULL)) {
		LOG_ERR("[lk_motor_err] rx handle Invalid arguments");
		return;
	}
	motor_lk_data_t *data = (motor_lk_data_t *)motor_dev->data; // 获取 motor 数据结构体指针
	const motor_lk_cfg_t *cfg =
		(const motor_lk_cfg_t *)motor_dev->config; // 获取 motor 配置结构体指针
	if (cfg == NULL || data == NULL) {
		LOG_ERR("[lk_motor_err] data and cfg struct is NULL");
		return;
	}

	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		LOG_ERR("[lk_motor_err] RTR frame received");
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock); // 加锁保护 motor_data
	data->motor_data.heartbeat_status.is_alive = true;
	/* 解析 CAN 帧数据 */
	if (cfg->control_mode == 0) // 处于多电机控制模式
	{
		int ret = motor_lk_multicontrol_iq(data, frame);
		if (ret != 0) {
			LOG_WRN("[lk_motor_err] Failed to handle multicontrol frame");
		}
	} else // 处于单电机控制模式
	{
		int cmd_id = frame->data[0];
		switch (cmd_id) {
		case READ_MOTOR_STATE1:
		case CLEAR_MOTOR_ERROR: {
			int ret = motor_lk_readstate1(data, frame);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readstate1 frame");
			}
			break;
		}
		case READ_MOTOR_STATE2:
		case OPEN_LOOP_CONTROL:
		case CLOSED_LOOP_CONTROL:
		case CLOSED_SPEED_CONTROL:
		case MULTI_POSITION_CONTROL1:
		case MULTI_POSITION_CONTROL2:
		case SINGLE_POSITION_CONTROL1:
		case SINGLE_POSITION_CONTROL2:
		case INCRE_POSITION_CONTROL1:
		case INCRE_POSITION_CONTROL2: {
			int ret = motor_lk_readstate2(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readstate2 frame");
			}
			break;
		}
		case READ_MOTOR_STATE3: {
			int ret = motor_lk_readstate3(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readstate3 frame");
			}
			break;
		}
		case READ_BRAKE_STATE: {
			int ret = motor_lk_readbreak(data, frame);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readbrakestate frame");
			}
			break;
		}
		case READ_CONTROL_PARAM: {
			int ret = motor_lk_readparam(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readcontrolparams frame");
			}
			break;
		}
		case READ_ENCODER: {
			int ret = motor_lk_readencoder(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readencoder frame");
			}
			break;
		}
		case READ_MULTI_ENCODER: {
			int ret = motor_lk_readmultiencoder(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readmultiencoder frame");
			}
			break;
		}
		case READ_SINGLE_ENCODER: {
			int ret = motor_lk_readsingleencoder(data, frame, cfg);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle readsingleencoder frame");
			}
			break;
		}
		case SET_ENCODER_ZERO: {
			int ret = motor_lk_setencoderzero(data, frame);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle setencoderzero frame");
			}
			break;
		}
		case SET_ENCODER_SELF: {
			int ret = motor_lk_setencoderself(data, frame);
			if (ret != 0) {
				LOG_WRN("[lk_motor_err] Failed to handle setencoderself frame");
			}
			break;
		}
		}
	}
	data->motor_data.heartbeat_status.heartbeat_tick = (uint64_t)k_uptime_get();
	k_spin_unlock(&data->lock, key);
}
#endif

#if defined(CONFIG_CAN_TX_MANAGER)
static int motor_lk_can_tx_fillbuffer_handler(struct can_frame *frame, void *user_data)
{
	const struct device *motor_dev = (const struct device *)user_data;
	if ((motor_dev == NULL) || (frame == NULL)) {
		LOG_ERR("[lk_motor_err] tx fillbuffer handle Invalid arguments");
		return -EINVAL;
	}

	motor_lk_data_t *data = (motor_lk_data_t *)motor_dev->data; // 获取 motor 数据结构体指针
	const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)motor_dev->config; // 获取 motor 配置结构体指针
	if (cfg == NULL || data == NULL || frame == NULL) {
		LOG_ERR("[lk_motor_err] data and cfg struct is NULL");
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock); // 加锁保护 motor_data
	switch(cfg->control_mode)
	{
		case 0:             // 多电机控制模式, 只有多电机控制模式下才需要特殊处理
		{
			frame->dlc = 8;
			frame->flags = 0;
			int diff = cfg->rx_id % 10;
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
			memcpy(&frame->data[idx], &data->motor_data.tx_data[0], 2);    
			k_spin_unlock(&data->lock, key);
			return 0;
		}
		case 1:                // 单电机控制模式，直接按字节发送
			memcpy(&frame->data[0], &data->motor_data.tx_data[0], 8);
			k_spin_unlock(&data->lock, key);
			return 0;
	}
	k_spin_unlock(&data->lock, key);
	return 0;
}
#endif
/**
 * @brief 单电机注册接口，通过 motor_id 识别电机实例，配置了can的接收过滤器
 *
 * @param dev
 * @return int
 */
static int motor_lk_can_register_motor(const struct device *dev)
{
	const motor_lk_cfg_t *cfg = dev->config;
	motor_lk_data_t *data = dev->data;

	if ((cfg == NULL) || (data == NULL)) {
		LOG_ERR("[lk_motor_err] register Invalid arguments");
		return -EINVAL;
	}

	if (!device_is_ready(cfg->can_dev)) {
		LOG_ERR("[lk_motor_err] CAN device not ready");
		return -ENODEV;
	}

	if (data->registered) {
		LOG_WRN("[lk_motor_err] motor already registered,please check your code");
		return -EALREADY;
	}
#if defined(CONFIG_CAN_RX_MANAGER)
	if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
		LOG_ERR("[lk_motor_err] RX manager not ready");
		return -ENODEV;
	}
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
	if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
		LOG_ERR("[lk_motor_err] TX manager not ready");
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
	rx_ret = can_rx_manager_register(cfg->rx_mgr, &filter, motor_lk_can_rx_handler,
					 (void *)dev);
	if (rx_ret < 0) {
		LOG_ERR("[lk_motor_err] Failed to register motor on RxManager: %d", rx_ret);
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
					 motor_lk_can_tx_fillbuffer_handler, (void *)dev);
	if (tx_ret < 0) {
		LOG_ERR("[lk_motor_err] Failed to register CAN TX filter: %d", tx_ret);
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
	data->motor_data.tx_data[0] = 0;
	data->motor_data.interface_ptr = (void *)cfg;
	data->motor_data.rx_data.valid_mask = 0U;
	data->motor_data.heartbeat_status.is_alive = false;
	data->motor_data.heartbeat_status.heartbeat_tick = 0;
	return 0;
}

/**
 * @brief 瓴控电机的心跳探测函数，只有在单电机控制模式下，主动发送心跳帧更新探测时间戳
 * 	  		在多电机控制模式下，直接等待接收回调更新心跳时间戳即可
 *
 * @param dev
 */
int motor_lk_can_heartbeat_probe(const struct device *dev)
{
	if (dev == NULL) {
		LOG_ERR("[lk_motor_err] heartbeat probe Invalid arguments");
		return -EINVAL;
	}

	motor_lk_data_t *data = dev->data;
	if (data == NULL) {
		LOG_ERR("[lk_motor_err] heartbeat probe data NULL");
		return -EINVAL;
	}

	const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)data->motor_data.interface_ptr;
	if (cfg == NULL) {
		LOG_ERR("[lk_motor_err] heartbeat probe cfg NULL");
		return -EINVAL;
	}

	// uint64_t now_probe_tick = (uint64_t)k_uptime_get();

	// 当处于单电机的控制模式下，发送读取电机状态1
	if(cfg->control_mode != 0)
	{
		struct can_frame hb_frame = {
			.dlc = 8,
			.flags = 0,
			.id = cfg->tx_id,
		};
		hb_frame.data[0] = 0x9c;
		int ret = can_send(cfg->can_dev, &hb_frame, K_MSEC(20), NULL, NULL);
		if (ret != 0) {
			LOG_ERR("[lk_motor_err] heartbeat probe send failed");
		}
		else{
			k_spinlock_key_t key = k_spin_lock(&data->lock);
			// data->motor_data.heartbeat_status.probe_tick = now_probe_tick;
			k_spin_unlock(&data->lock, key);
		}
	}
	// TODO: 多电机下没有实现这种主动探测心跳，1. 混合命令没有验证 2. 发送管理器目前不支持只发一次报文帧的多电机合并
	return 0;
}

/**
 * @brief 瓴控电机的心跳状态更新函数，供应用层/中间件调用以获取最新心跳状态.
 * 			注意，probe_tick的值可能是在心跳主动探测时更新，也可能是在发送填充中更新
 *
 * @param dev
 * @return int
 */
int motor_lk_update_heartbeat_status(const struct device *dev)
{
	if (dev == NULL) {
		LOG_ERR("[lk_motor_err] update heartbeat Invalid arguments");
		return -EINVAL;
	}

	motor_lk_data_t *data = dev->data;
	if (data == NULL) {
		LOG_ERR("[lk_motor_err] update heartbeat data NULL");
		return -EINVAL;
	}
	uint64_t elapsed = 0;
	const motor_lk_cfg_t *cfg = (const motor_lk_cfg_t *)data->motor_data.interface_ptr;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint64_t last_tick = data->motor_data.heartbeat_status.heartbeat_tick;
	// uint64_t probe_tick = data->motor_data.heartbeat_status.probe_tick;
	uint64_t current_tick = (uint64_t)k_uptime_get();
	bool prev_alive = data->motor_data.heartbeat_status.is_alive;

	/* 尚未收到过任何帧时，每隔 1s 告警一次，避免静默失败。 */
	if (last_tick == 0U) {
		LOG_WRN_RATELIMIT_RATE(1000,
			"[lk_motor_err] motor offline! no first CAN frame received (%s, rx=0x%03x)",
			(cfg != NULL && cfg->motor_label != NULL) ? cfg->motor_label : "unknown",
			(cfg != NULL) ? (unsigned int)cfg->rx_id : 0U);
		data->motor_data.heartbeat_status.is_alive = false;
		k_spin_unlock(&data->lock, key);
		return 0;
	}
	// TODO: 原本是打算设定一个probe机制，但是会导致heartbeat>probe,因为同时接收也会更新heartbeat时间戳
	// if (cfg->control_mode != 0) { // 单电机控制模式，心跳由主动探测触发，探测时间戳为准
	// 	elapsed = probe_tick - last_tick;
	// }
	// else { // 多电机控制模式，心跳由接收回调触发，心跳时间戳为准
	// 	elapsed = current_tick - last_tick;
	// }
	elapsed = current_tick - last_tick;
	/* 如果超过阈值没有收到心跳，则认为电机掉线：清零接收值并在离线边沿告警一次 */
	if (elapsed > (uint64_t)CONFIG_MOTOR_HEARTBEAT_OFFLINE_TIMEOUT_MS) {
		data->motor_data.heartbeat_status.is_alive = false;

		/* 只有从在线->离线时，才清零并告警；避免每次轮询刷屏 */
		if (prev_alive) {
			memset(&data->motor_data.rx_data, 0, sizeof(data->motor_data.rx_data));
			LOG_ERR("[lk_motor_err] motor offline (%s, rx=0x%03x): no CAN frames for "
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

/**
 * @brief 暴露给中间件的修改发送频率接口，修改后会立即生效
 *
 * @param dev
 * @param new_feq
 * @return int
 */
static int motor_lk_can_change_tx_feq(const struct device *dev, uint16_t new_feq)
{
    motor_lk_data_t *data = dev->data;
    if (data == NULL) {
        LOG_ERR("[lk_motor_err] change Tx feq Invalid arguments");
        return -EINVAL;
    }
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->Tx_feq = new_feq;
    k_spin_unlock(&data->lock, key);
    LOG_WRN("[lk_motor] Tx frequency changed to %u Hz", new_feq);
    return 0;
}

/**
 * @brief 获取电机心跳状态接口，供应用层/中间件调用
 *
 * @param dev
 * @return int 1: alive, 0: not alive, <0: error code
 */
static int motor_lk_can_get_heartbeat_status(const struct device *dev)
{
    int ret = motor_lk_update_heartbeat_status(dev);
    if (ret < 0) {
        return ret;
    }

    motor_lk_data_t *data = dev->data;
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
static const smotor_receive_data_t *motor_lk_can_get_rxdata(const struct device *dev)
{
    motor_lk_data_t *data = dev->data;
    if (data == NULL) {
        LOG_WRN("[lk_motor_err] get_rxdata dev NULL");
        return NULL;
    }

    return &data->motor_data.rx_data;
}

static const motor_lk_single_data_t *motor_lk_can_get_single_data(const struct device *dev)
{
	motor_lk_data_t *data = dev->data;
	if (data == NULL) {
		LOG_WRN("[lk_motor_err] get_single_data dev NULL");
		return NULL;
	}

	return data->single_data;
}


const motor_driver_api_t motor_lk_can_api = {
    .register_motor = motor_lk_can_register_motor,
    .change_tx_feq = motor_lk_can_change_tx_feq,
    .torque_control = motor_lk_multi_torquecontrol,
    .get_heartbeat_status = motor_lk_can_get_heartbeat_status,
    .get_rxdata = motor_lk_can_get_rxdata,
	.clear_error = motor_lk_clearerror,
	.disable = motor_lk_disable,
	.enable = motor_lk_enable,
	.stop = motor_lk_stop,
	.lk_api.get_single_data = motor_lk_can_get_single_data,
	.lk_api.single_openloop_control = motor_lk_single_openloopcontrol,
	.lk_api.single_closedloop_control = motor_lk_single_closedloopcontrol,
	.lk_api.single_speedcontrol = motor_lk_single_speedcontrol,
	.lk_api.single_mulposctrl1 = motor_lk_single_mulposcontrol1,
	.lk_api.single_mulposctrl2 = motor_lk_single_mulposcontrol2,
	.lk_api.single_sigposctrl1 = motor_lk_single_sigposcontrol1,
	.lk_api.single_sigposctrl2 = motor_lk_single_sigposcontrol2,
	.lk_api.single_increposctrl1 = motor_lk_single_increposcontrol1,
	.lk_api.single_increposctrl2 = motor_lk_single_increposcontrol2,
	.lk_api.multi_speedcontrol = motor_lk_multi_speedcontrol,
	.lk_api.multi_positcontrol = motor_lk_multi_positcontrol,
	.lk_api.multi_mixcontrol = motor_lk_multi_mixcontrol,
};

/**
 * @brief lk电机实例的初始化
 *
 * @param dev
 * @return int
 */
int motor_lk_can_init(const struct device *dev)
{
	const motor_lk_cfg_t *cfg = dev->config;
	motor_lk_data_t *data = dev->data;

	if (!device_is_ready(cfg->can_dev)) {
		return -ENODEV;
	}

	int start_ret = can_start(cfg->can_dev);
	if ((start_ret < 0) && (start_ret != -EALREADY)) {
		LOG_ERR("[lk_motor_err] Failed to start CAN device, error: %d", start_ret);
		return start_ret;
	}

#if defined(CONFIG_CAN_RX_MANAGER)
	if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
		__ASSERT(false, "[lk_motor_err] RX manager not ready for device %s", dev->name);
		return -ENODEV;
	}
#endif
#if defined(CONFIG_CAN_TX_MANAGER)
	if ((cfg->tx_mgr == NULL) || !device_is_ready(cfg->tx_mgr)) {
		__ASSERT(false, "[lk_motor_err] TX manager not ready for device %s", dev->name);
		return -ENODEV;
	}
#endif

	data->registered = false;
	memset(&data->motor_data, 0, sizeof(data->motor_data));
	data->motor_data.interface_ptr = (void *)cfg;
	data->motor_data.rx_data.valid_mask = 0U;
	data->motor_data.heartbeat_status.is_alive = false;
	data->motor_data.heartbeat_status.heartbeat_tick = 0;

#if defined(CONFIG_MOTOR_HEARTBEAT_AUTOCHECK)
	data->dev_self = dev;
	k_work_init_delayable(&data->hb_work, motor_lk_hb_work_handler);
	(void)k_work_schedule(&data->hb_work, K_MSEC(CONFIG_MOTOR_HEARTBEAT_POLL_PERIOD_MS));
#endif

	return 0;
}

/* ---------- Devicetree helpers ---------- */

/* 获取 control-mode enum 的索引；未配置则返回 -1 */
#define MOTOR_LK_CONTROL_MODE(inst)                                                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, control_mode), (DT_INST_ENUM_IDX(inst, control_mode)), (-1))

#define MOTOR_LK_TYPE(inst)                                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, motor_type), (DT_INST_ENUM_IDX(inst, motor_type)), (-1))

#define MOTOR_LK_DEFINE(inst)                                                                     \
	static motor_lk_single_data_t motor_lk_single_data_##inst = {                               \
	};                                                                                         \
	static const motor_lk_cfg_t motor_lk_cfg_##inst = {                                      \
		.tx_id = (uint16_t)DT_INST_PROP(inst, tx_id),                                      \
		.rx_id = (uint16_t)DT_INST_PROP(inst, rx_id),                                      \
		.motor_label = DT_INST_PROP(inst, label),                                          \
		.motor_type = (int8_t)MOTOR_LK_TYPE(inst),                                        \
		.control_mode = (int8_t)MOTOR_LK_CONTROL_MODE(inst),                              \
		.motor_encoder = (uint16_t)DT_INST_PROP(inst, motor_encoder),                      \
		.transmission_ratio = (uint8_t)DT_INST_PROP(inst, motor_transmission_ratio),       \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                          \
		IF_ENABLED(CONFIG_CAN_RX_MANAGER, ( \
            .rx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rx_manager), \
                          (DEVICE_DT_GET(DT_INST_PHANDLE(inst, rx_manager))), (NULL)), \
	        )) \
		IF_ENABLED(CONFIG_CAN_TX_MANAGER, ( \
            .tx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, tx_manager), \
                          (DEVICE_DT_GET(DT_INST_PHANDLE(inst, tx_manager))), (NULL)), \
	        )) \
	};                                                                                         \
	static motor_lk_data_t motor_lk_data_##inst = {                                          \
		.single_data = (DT_INST_ENUM_IDX(inst, control_mode) == 0) ? NULL :                  \
				   &motor_lk_single_data_##inst,                                              \
		.Tx_feq = (uint16_t)DT_INST_PROP(inst, tx_feq),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, motor_lk_can_init, NULL, &motor_lk_data_##inst,              \
			      &motor_lk_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,      \
			      &motor_lk_can_api);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(MOTOR_LK_DEFINE)
#endif
