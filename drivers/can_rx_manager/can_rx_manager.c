/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#define DT_DRV_COMPAT rp_can_rx_manager

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <string.h>

#include <drivers/can_rx_manager.h>

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(can_rx_manager);


struct rp_can_rx_listener {
	bool used;
	struct can_filter filter;
	can_rx_handler_t handler;
	void *user_data;
};

struct rp_can_rx_manager_cfg {
	const struct device *can_dev;
	struct k_msgq *rx_msgq;
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
};

struct rp_can_load_calculate
{
	/* cumulative bit counters for load calculation */
	atomic_t rx_bits_nominal; /* bits counted at nominal (arbitration) rate */
	atomic_t rx_bits_data;	  /* bits counted at data (BRS) rate for FD data phase */
	uint64_t last_load_ts_ms; /* timestamp of last load snapshot */
	uint64_t last_load_bits_nominal;
	uint64_t last_load_bits_data;
};


struct rp_can_rx_manager_data {
	struct rp_can_rx_listener listeners[CONFIG_CAN_RX_MANAGER_MAX_LISTENERS];
	struct k_thread rx_thread;
	int hw_filter_id;
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	atomic_t rx_dropped;
	atomic_t rx_queued;
	uint32_t last_reported_drops;
#endif
	struct rp_can_load_calculate load_calc;
	double can_load;
};

struct rp_can_rx_msg {
	const struct device *mgr;
	struct can_frame frame;
};

/* Must use K_MSGQ_DEFINE with sizeof(rp_can_rx_msg), NOT CAN_MSGQ_DEFINE which
 * only allocates sizeof(struct can_frame) per element and would cause overflow. */
K_MSGQ_DEFINE(rp_can_rx_shared_msgq, sizeof(struct rp_can_rx_msg), CONFIG_CAN_RX_MANAGER_RX_MSGQ_LEN, 4);
K_THREAD_STACK_DEFINE(rp_can_rx_shared_stack, CONFIG_CAN_RX_MANAGER_RX_STACK_SIZE);
static struct k_thread rp_can_rx_shared_thread_data;
static atomic_t rp_can_rx_shared_started = ATOMIC_INIT(0);

static void rp_can_rx_isr_cb(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(can_dev);

	const struct device *mgr = (const struct device *)user_data;
	if ((mgr == NULL) || (frame == NULL)) {
		LOG_ERR("[can_rx_manager] Invalid ISR callback parameters");
		return;
	}

	struct rp_can_rx_msg msg;
	msg.mgr = mgr;
	msg.frame = *frame;

	/* Skip RTR frames by default */
	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		return;
	}

	int ret = k_msgq_put(&rp_can_rx_shared_msgq, &msg, K_NO_WAIT);
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	/* update per-manager counters for monitoring */
	struct rp_can_rx_manager_data *data = mgr->data;
	if (data != NULL) {
		if (ret == 0) {
			(void)atomic_inc(&data->rx_queued);
		} else {
			(void)atomic_inc(&data->rx_dropped);
		}
	}
#endif
	/* accumulate bus-bit estimates for load calculation (per-manager counters) */
	struct rp_can_rx_manager_data *mdata = mgr->data;
	if (mdata != NULL) {
		/* estimate bits for this frame: nominal bits and data-phase bits */
		uint32_t nominal_bits = 0;
		uint32_t data_bits = 0;
		/* helper: convert DLC to payload length in bytes */
		static const uint8_t dlc_to_len_map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
		uint8_t len = (frame->dlc < 16) ? dlc_to_len_map[frame->dlc] : 0;

		bool is_ext = (frame->flags & CAN_FRAME_IDE) != 0U;
		bool is_fd = (frame->flags & CAN_FRAME_FDF) != 0U;
		bool is_brs = (frame->flags & CAN_FRAME_BRS) != 0U;

		if (!is_fd) {
			/* Classic CAN: approximate baseline overhead
			 * standard ID: ~47 bits overhead, extended ID: ~67 bits
			 */
			nominal_bits = (is_ext ? 67U : 47U) + (uint32_t)len * 8U;
			data_bits = 0U;
		} else {
			/* CAN-FD: arbitration/header transmitted at nominal rate; data phase at data rate when BRS set
			 * Use conservative estimates: header overhead similar to classic; data CRC ~17/21 bits depending on length
			 */
			nominal_bits = (is_ext ? 67U : 47U);
			/* CRC length heuristic */
			uint32_t crc_bits = (len <= 16U) ? 17U : 21U;
			data_bits = (uint32_t)len * 8U + crc_bits;
			/* If BRS not set, data phase also happens at nominal rate -> fold into nominal_bits */
			if (!is_brs) {
				nominal_bits += data_bits;
				data_bits = 0U;
			}
		}

		(void)atomic_add(&mdata->load_calc.rx_bits_nominal, (int)nominal_bits);
		(void)atomic_add(&mdata->load_calc.rx_bits_data, (int)data_bits);
	}
	ARG_UNUSED(ret);
}

/**
 * @brief Match a CAN frame against a software filter (standard/extended ID supported)
 *
 * @param filter Software filter to match against
 * @param frame  Received CAN frame
 * @return true  Frame matches the filter
 * @return false Frame does not match
 */
static bool rp_can_rx_match(const struct can_filter *filter, const struct can_frame *frame)
{
	uint32_t frame_id = frame->id;
	uint32_t filter_id = filter->id;

	if (((frame->flags & CAN_FRAME_IDE) != 0U) != ((filter->flags & CAN_FILTER_IDE) != 0U)) {
		return false;
	}

	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		frame_id &= CAN_EXT_ID_MASK;
		filter_id &= CAN_EXT_ID_MASK;
	} else {
		frame_id &= CAN_STD_ID_MASK;
		filter_id &= CAN_STD_ID_MASK;
	}
	/* Compute masked values and log them for debugging */
	uint32_t masked_frame = frame_id & filter->mask;
	uint32_t masked_filter = filter_id & filter->mask;
	if (masked_frame == masked_filter) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Shared RX processing thread for all CAN manager instances
 *
 * @param p1 Unused
 * @param p2 Unused
 * @param p3 Unused
 */
static void rp_can_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct rp_can_rx_msg msg;

	while (true) {
		int ret = k_msgq_get(&rp_can_rx_shared_msgq, &msg, K_FOREVER);  /* Block until a CAN frame is available in the queue */
		if (ret != 0) {
			continue;
		}

		/* Process this message and try to drain queue to reduce wakeups,
		 * but limit number processed per wake to avoid unbounded stack/CPU use.
		 */
		int batch = 0;
		for (;;) {
			struct rp_can_rx_manager_data *data = NULL;
			if (msg.mgr != NULL) {
				data = msg.mgr->data;
			}
			if (data != NULL) {
				struct can_frame *frame = &msg.frame;
				for (int i = 0; i < CONFIG_CAN_RX_MANAGER_MAX_LISTENERS; i++) {		/* Iterate all listener slots; invoke handler if filter matches */
					struct rp_can_rx_listener *lst = &data->listeners[i];
					if (!lst->used) {
						continue;
					}
					if (!rp_can_rx_match(&lst->filter, frame)) {
						continue;
					}
					lst->handler(frame, lst->user_data);
				}
			}

	#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
			/* Report drops per-manager when threshold reached */
			if (data != NULL) {
				uint32_t drops = (uint32_t)atomic_get(&data->rx_dropped);
				uint32_t last = data->last_reported_drops;
				if (drops > last) {
					uint32_t delta = drops - last;
					if ((uint32_t)CONFIG_CAN_RX_MANAGER_MSGQ_WARN_EVERY_N_DROPS > 0 &&
					    delta >= (uint32_t)CONFIG_CAN_RX_MANAGER_MSGQ_WARN_EVERY_N_DROPS) {
						LOG_WRN("can_rx_manager(%p): %u frames dropped (cumulative)", msg.mgr, drops);
						data->last_reported_drops = drops;
					}
				}
			}
	#endif

			batch++;
			if (batch >= CONFIG_CAN_RX_MANAGER_BATCH_LIMIT) {
				//LOG_DBG("RX mgr batch limit %d reached, may be busy", (int)CONFIG_CAN_RX_MANAGER_BATCH_LIMIT);
				k_yield();
				break;
			}

			if (k_msgq_get(&rp_can_rx_shared_msgq, &msg, K_NO_WAIT) != 0) {
				break; /* no more messages immediately available */
			}
		}
	}
}

/**
 * @brief Register a software RX listener. All CAN devices must register here before
 *        receiving frames through the manager.
 *
 * @param mgr       CAN RX manager device
 * @param filter    Software filter for frame matching
 * @param handler   Callback invoked when a matching frame is received
 * @param user_data Opaque pointer passed to the handler
 * @return int      Listener ID on success, negative error code on failure
 */
int rp_can_rx_manager_register(const struct device *mgr, const struct can_filter *filter,
			      can_rx_handler_t handler, void *user_data)
{
	if ((mgr == NULL) || (filter == NULL) || (handler == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(mgr)) {
		return -ENODEV;
	}

	struct rp_can_rx_manager_data *data = mgr->data;
	if (data == NULL) {
		return -EINVAL;
	}

	for (int i = 0; i < CONFIG_CAN_RX_MANAGER_MAX_LISTENERS; i++) {
		if (data->listeners[i].used) {
			continue;
		}
		data->listeners[i].used = true;
		data->listeners[i].filter = *filter;
		data->listeners[i].handler = handler;
		data->listeners[i].user_data = user_data;
		LOG_INF("can_rx_manager: registered listener id=%d filter_id=0x%03x mask=0x%03x", i, (unsigned int)filter->id, (unsigned int)filter->mask);
		return i;
	}

	return -ENOSPC;
}

/**
 * @brief Unregister a previously registered listener
 *
 * @param mgr         CAN RX manager device
 * @param listener_id Listener ID returned by rp_can_rx_manager_register()
 * @return int        0 on success, negative error code on failure
 */
int rp_can_rx_manager_unregister(const struct device *mgr, int listener_id)
{
	if ((mgr == NULL) || (listener_id < 0) || (listener_id >= CONFIG_CAN_RX_MANAGER_MAX_LISTENERS)) {
		return -EINVAL;
	}
	if (!device_is_ready(mgr)) {
		return -ENODEV;
	}

	struct rp_can_rx_manager_data *data = mgr->data;
	if (data == NULL) {
		return -EINVAL;
	}

	if (!data->listeners[listener_id].used) {
		return -ENOENT;
	}

	data->listeners[listener_id].used = false;
	data->listeners[listener_id].handler = NULL;
	data->listeners[listener_id].user_data = NULL;
	memset(&data->listeners[listener_id].filter, 0, sizeof(data->listeners[listener_id].filter));
	return 0;
}


/**
 * @brief CAN RX manager driver initialization
 *
 * @param dev Device instance
 * @return int 0 on success, negative error code on failure
 */
static int rp_can_rx_manager_init(const struct device *dev)
{
	const struct rp_can_rx_manager_cfg *cfg = dev->config;
	struct rp_can_rx_manager_data *data = dev->data;

	if ((cfg == NULL) || (data == NULL)) {
		LOG_ERR("[can_rx_manager] init failed - invalid config or data");
		return -EINVAL;
	}
	if (!device_is_ready(cfg->can_dev)) {
		LOG_ERR("[can_rx_manager] init failed - CAN device not ready");
		return -ENODEV;
	}

	int ret = can_start(cfg->can_dev); // 启动 CAN 设备,
	if ((ret < 0) && (ret != -EALREADY))
	{
		return ret;
	}

	/* One broad hardware filter: accept all standard IDs into manager msgq */
	const struct can_filter hw = {
		.id = 0,
		.mask = 0,
		.flags = 0,
	};

	/* Register a single ISR callback that will enqueue into the shared msgq */
	ret = can_add_rx_filter(cfg->can_dev, rp_can_rx_isr_cb, (void *)dev, &hw);
	if (ret < 0) {
		return ret;
	}
	data->hw_filter_id = ret;

	/* Start the shared processing thread once */
	/* Initialize monitoring counters if enabled */
#if defined(CONFIG_CAN_RX_MANAGER_MSGQ_MONITOR)
	atomic_set(&data->rx_dropped, 0);
	atomic_set(&data->rx_queued, 0);
	data->last_reported_drops = 0;
#endif
	/* The shared thread only needs to be started once across all instances; guard with atomic CAS */
	if (atomic_cas(&rp_can_rx_shared_started, 0, 1)) {
		k_thread_create(&rp_can_rx_shared_thread_data, rp_can_rx_shared_stack,
				K_THREAD_STACK_SIZEOF(rp_can_rx_shared_stack), rp_can_rx_thread,
				NULL, NULL, NULL, CONFIG_CAN_RX_MANAGER_RX_THREAD_PRIO, 0, K_NO_WAIT);
	}

	return 0;
}

/**
 * @brief
 *
 * @param mgr
 * @param nominal_bitrate_bps Bitrate used for arbitration/CRC/control (bps)
 * @param data_bitrate_bps Bitrate used for FD data phase when BRS is set (bps). If 0, FD data is assumed to use nominal rate.
 * @return load percentage in range [0.0 .. 100.0], negative on error.
 */
static float rp_can_rx_manager_calculate_load(const struct device *mgr, uint32_t nominal_bitrate_bps, uint32_t data_bitrate_bps)
{
	if (mgr == NULL) {
		return -EINVAL;
	}
	struct rp_can_rx_manager_data *data = mgr->data;
	if (data == NULL) {
		return -EINVAL;
	}

	uint64_t now_ms = k_uptime_get();

	/* Read 32-bit atomic counters (they may wrap); handle wrap-around below */
	uint32_t cur_nom32 = (uint32_t)atomic_get(&data->load_calc.rx_bits_nominal);
	uint32_t cur_dat32 = (uint32_t)atomic_get(&data->load_calc.rx_bits_data);

	uint64_t last_nom = data->load_calc.last_load_bits_nominal;
	uint64_t last_dat = data->load_calc.last_load_bits_data;
	uint64_t last_ts = data->load_calc.last_load_ts_ms;

	/* On first call initialize snapshots */
	if (last_ts == 0) {
		data->load_calc.last_load_bits_nominal = cur_nom32;
		data->load_calc.last_load_bits_data = cur_dat32;
		data->load_calc.last_load_ts_ms = now_ms;
		return 0.0f;
	}

	/* Compute deltas handling 32-bit wrap-around of the atomic counters.
	 * `last_nom`/`last_dat` store the previous low-32 snapshot value.
	 */
	uint32_t last_nom32 = (uint32_t)(last_nom & 0xFFFFFFFFU);
	uint32_t last_dat32 = (uint32_t)(last_dat & 0xFFFFFFFFU);

	uint64_t delta_nom;
	if (cur_nom32 >= last_nom32) {
		delta_nom = (uint64_t)(cur_nom32 - last_nom32);
	} else {
		/* wrapped */
		delta_nom = (uint64_t)cur_nom32 + (uint64_t)UINT32_MAX + 1ULL - (uint64_t)last_nom32;
	}

	uint64_t delta_dat;
	if (cur_dat32 >= last_dat32) {
		delta_dat = (uint64_t)(cur_dat32 - last_dat32);
	} else {
		delta_dat = (uint64_t)cur_dat32 + (uint64_t)UINT32_MAX + 1ULL - (uint64_t)last_dat32;
	}
	uint64_t delta_ms = (now_ms > last_ts) ? (now_ms - last_ts) : 0;
	if (delta_ms == 0) {
		return 0.0f;
	}

	if (nominal_bitrate_bps == 0) {
		return -EINVAL;
	}
	if (data_bitrate_bps == 0) {
		data_bitrate_bps = nominal_bitrate_bps;
	}

	/* Time on bus in seconds = bits_at_nominal/nominal_bps + bits_at_data/data_bps */
	double t_nom = (double)delta_nom / (double)nominal_bitrate_bps;
	double t_dat = (double)delta_dat / (double)data_bitrate_bps;
	double elapsed_s = (double)delta_ms / 1000.0;

	double load = 0.0;
	if (elapsed_s > 0.0) {
		load = (t_nom + t_dat) / elapsed_s * 100.0;
		if (load < 0.0) {
			load = 0.0;
		} else if (load > 100.0) {
			load = 100.0;
		}
	}

	/* update snapshots (store low-32 snapshots so wrap handling works next call) */
	data->load_calc.last_load_bits_nominal = (uint64_t)cur_nom32;
	data->load_calc.last_load_bits_data = (uint64_t)cur_dat32;
	data->load_calc.last_load_ts_ms = now_ms;

	return (float)load;
}

static const struct can_rx_manager_api rp_can_rx_mgr_api = {
	.register_listener = rp_can_rx_manager_register,
	.unregister_listener = rp_can_rx_manager_unregister,
	.calculate_load = rp_can_rx_manager_calculate_load,
};

#define RP_CAN_RX_MGR_DEFINE(inst)                                                              \
	static const struct rp_can_rx_manager_cfg rp_can_rx_mgr_cfg_##inst = {                      \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                               \
		.rx_msgq = &rp_can_rx_shared_msgq,                                                      \
		.rx_stack = rp_can_rx_shared_stack,                                                     \
		.rx_stack_size = K_THREAD_STACK_SIZEOF(rp_can_rx_shared_stack),                         \
	};                                                                                          \
	static struct rp_can_rx_manager_data rp_can_rx_mgr_data_##inst;                             \
	DEVICE_DT_INST_DEFINE(inst, rp_can_rx_manager_init, NULL, &rp_can_rx_mgr_data_##inst,       \
						  &rp_can_rx_mgr_cfg_##inst, POST_KERNEL, CONFIG_CAN_RX_MANAGER_INIT_PRIORITY, &rp_can_rx_mgr_api);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_RX_MGR_DEFINE)

