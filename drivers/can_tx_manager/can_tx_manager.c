/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

 #define DT_DRV_COMPAT rp_can_tx_manager

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <drivers/can_tx_manager.h>

#include <string.h>

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(can_tx_manager);

#define CAN_TX_MGR_TICK_MS 1U

/* maximum allowed periodic transmission frequency */
#define CAN_TX_MGR_MAX_FREQ (1000U / CAN_TX_MGR_TICK_MS)

#define _RP_CAN_TX_MGR_DEV_PTR(inst) DEVICE_DT_INST_GET(inst),

typedef struct rp_can_tx_cfg {
    const struct device *can_dev;
} rp_can_tx_cfg_t;

typedef struct device_sender_cfg {
    uint16_t tx_id;
    uint16_t rx_id;
    bool used;
    void *user_data;
    tx_fillbuffer_cb_t fill_buffer_cb;       /* callback used to fill transmit data */
} device_sender_cfg_t;

typedef struct rp_can_item {
    struct can_frame frame;
    uint16_t frequency;             /* transmit frequency in Hz; 0 means event‑driven */
    uint16_t interval;              /* interval in ticks (computed during registration) */
    uint16_t tick_counter;          /* accumulated ticks until next send */
} rp_can_item_t;

typedef struct rp_can_tx_data
{
    device_sender_cfg_t sender_list[CONFIG_MAX_DEVICE_SENDERS];
    rp_can_item_t can_items[CONFIG_MAX_CAN_FRAMES];    /* managed CAN frames, statically allocated */
    struct k_mutex lock;                         /* mutex protecting data */
    uint8_t frame_num;                          /* number of active frames */
} rp_can_tx_data_t;

/**
 * @brief Initialize TX manager state and mutex
 *
 * Sets up internal arrays and prepares the mutex protecting
 * runtime data.  The configuration pointer is not used during
 * initialization but is validated for NULL.
 *
 * @param dev Pointer to the device instance being initialized
 * @return 0 on success, negative error code on failure
 */
int rp_can_tx_manager_init(const struct device *dev)
{
    const rp_can_tx_cfg_t *cfg = dev->config;
    struct rp_can_tx_data *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager init failed - invalid config or data");
        return -EINVAL;
    }

    (void)cfg;
    memset(&data->sender_list, 0, sizeof(data->sender_list));
    memset(&data->can_items, 0, sizeof(data->can_items));
    data->frame_num = 0;
    k_mutex_init(&data->lock);                                         /* initialize mutex */

    return 0;
}

/**
 * @brief Register a transmitter with the TX manager
 *
 * A caller provides an ID, DLC/flags, optional frequency, and a
 * callback to fill the frame before each transmission.  Multiple
 * transmitters may share the same tx_id; they are de-duplicated in the
 * manager and their callbacks will all be invoked when the frame is
 * prepared.
 *
 * @param mgr Pointer to the CAN TX manager device
 * @param tx_id CAN identifier for outgoing frames
 * @param rx_id Reserved for future use (receive ID)
 * @param dlc Data length code for the frame (usually 8)
 * @param flags CAN frame flags (0 for standard frame)
 * @param frequency Transmit rate in Hz (0 means event-driven)
 * @param fill_buffer_cb Callback invoked to populate the frame payload
 * @param user_data Opaque pointer passed to the callback
 * @return non‑negative registration index on success, negative error code
 */
int rp_can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id,
                                uint8_t dlc, uint8_t flags, uint16_t frequency,
                                tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    if (!device_is_ready(mgr))
    {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    struct rp_can_tx_data *data = mgr->data;
    if (data == NULL)
    {
        LOG_ERR("[can_tx_manager]CAN TX manager data is NULL");
        return -EINVAL;
    }

    /* limit frequency to what the periodic thread can support */
    if (frequency > CAN_TX_MGR_MAX_FREQ) {
        LOG_ERR("[can_tx_manager]Invalid frequency %d Hz (max %u Hz)", frequency, CAN_TX_MGR_MAX_FREQ);
        return -EINVAL;
    }
    /* protect against concurrent access: hold lock until returning */
    k_mutex_lock(&data->lock, K_FOREVER);

    /* check whether the same tx_id is already registered */
    bool found = false;
    for (int f = 0; f < data->frame_num; f++) {
        if (data->can_items[f].frame.id == tx_id) {
            if (data->can_items[f].frequency != frequency) {
                LOG_ERR("[can_tx_manager]Cannot register same tx_id 0x%03x with different frequency (existing %d Hz, new %d Hz)", tx_id, data->can_items[f].frequency, frequency);
                k_mutex_unlock(&data->lock);
                return -EINVAL;
            }
            found = true;
            break;
        }
    }

    if (!found) {
        if (data->frame_num >= CONFIG_MAX_CAN_FRAMES) {
            LOG_ERR("[can_tx_manager]No space left for CAN frames");
            k_mutex_unlock(&data->lock);
            return -ENOSPC;
        }
        memset(&data->can_items[data->frame_num], 0, sizeof(rp_can_item_t));
        data->can_items[data->frame_num].frame.id = tx_id;
        data->can_items[data->frame_num].frame.dlc = dlc;
        data->can_items[data->frame_num].frame.flags = flags;
        data->can_items[data->frame_num].frequency = frequency;
        if (frequency > 0) {
            uint32_t ms_per_cycle = 1000U / frequency;
            data->can_items[data->frame_num].interval = (uint16_t)DIV_ROUND_UP(ms_per_cycle, CAN_TX_MGR_TICK_MS);
        }
        else {
            data->can_items[data->frame_num].interval = 0;
        }
        data->frame_num++;
    }
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (data->sender_list[i].used) {
            continue;
        }
        data->sender_list[i].used = true;
        data->sender_list[i].tx_id = tx_id;
        data->sender_list[i].rx_id = rx_id;
        data->sender_list[i].user_data = user_data;
        data->sender_list[i].fill_buffer_cb = fill_buffer_cb;
        k_mutex_unlock(&data->lock);
        return i; /* return allocated index */
    }

    /* no free slot available */
    k_mutex_unlock(&data->lock);
    return -ENOSPC;
}

/**
 * @brief Unregister a previously registered transmitter
 *
 * If the supplied tx_id has no remaining senders after this call,
 * the manager also removes the associated frame entry.
 *
 * @param mgr CAN TX manager device
 * @param tx_id Transmit ID to unregister
 * @param rx_id Must match the one used at registration
 * @return 0 on success or negative error if not found
 */
int rp_can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id)
{
    if (!device_is_ready(mgr))
    {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    struct rp_can_tx_data *data = mgr->data;
    if (data == NULL)
    {
        LOG_ERR("[can_tx_manager]CAN TX manager data is NULL");
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /* 1. unregister sender with given tx_id */
    bool found_sender = false;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (!data->sender_list[i].used) {
            continue;
        }
        if (data->sender_list[i].tx_id == tx_id && data->sender_list[i].rx_id == rx_id) {
            data->sender_list[i].used = false;
            data->sender_list[i].tx_id = 0;
            data->sender_list[i].rx_id = 0;
            data->sender_list[i].user_data = NULL;
            data->sender_list[i].fill_buffer_cb = NULL;
            found_sender = true;
            break;
        }
    }

    if (!found_sender) {
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }

    /* 2. check if other active senders exist for this tx_id */
    bool other_senders_exist = false;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (data->sender_list[i].used && data->sender_list[i].tx_id == tx_id) {
            other_senders_exist = true;
            break;
        }
    }

    /* 3. if no other senders exist, delete the associated CAN frame */
    if (!other_senders_exist) {
        for (int f = 0; f < data->frame_num; f++) {
            if (data->can_items[f].frame.id == tx_id) {
                /* remove the entry by shifting the tail down; memmove would
                 * express this intent more clearly. */
                if (f < data->frame_num - 1) {
                    memmove(&data->can_items[f], &data->can_items[f + 1],
                            (data->frame_num - f - 1) * sizeof(rp_can_item_t));
                }
                data->frame_num--;
                LOG_INF("[can_tx_manager]CAN frame tx_id 0x%03x deleted (no more senders)", tx_id);
                break;
            }
        }
    }

    k_mutex_unlock(&data->lock);
    return 0; /* successfully unregistered */

}

/**
 * @brief Invoke registered callbacks to populate CAN transmit buffer for use by send routines
 *
 * @param tx_id
 * @param frame
 * @param data
 * @param user_data
 * @return int
 */
static int rp_can_tx_fillbuffer(uint16_t tx_id, struct can_frame *frame, rp_can_tx_data_t *data)
{
    if (frame == NULL || data == NULL) {
        LOG_ERR("[can_tx_manager]Invalid frame or data pointer");
        return -EINVAL;
    }

    /* call every matching registration callback in turn */
    int cb_count = 0;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (!data->sender_list[i].used) {
            continue;
        }
        if (data->sender_list[i].tx_id != tx_id) {
            continue;
        }
        if (data->sender_list[i].fill_buffer_cb == NULL) {
            continue;
        }
        cb_count++;
        int ret = data->sender_list[i].fill_buffer_cb(frame, data->sender_list[i].user_data);
        if (ret != 0) {
            LOG_ERR("[can_tx_manager]Fill buffer callback failed for tx_id 0x%03x, err %d", tx_id, ret);
            return ret;
        }
    }

    if (cb_count == 0)
    {
        LOG_ERR("[can_tx_manager]No fill buffer callback for tx_id 0x%03x", tx_id);
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Send a CAN frame through the TX manager
 *
 * @param mgr CAN TX manager device
 * @param timeout send timeout
 * @param callback completion callback
 * @param tx_filter_id transmit ID (returned by register)
 * @param user_data user data for callback
 * @return int
 */
int rp_can_tx_manager_send(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data)
{
    if (mgr == NULL) {
        LOG_ERR("[can_tx_manager]CAN TX manager device is NULL");
        return -EINVAL;
    }

    const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
    rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
    if (cfg == NULL || cfg->can_dev == NULL || data == NULL) {
        LOG_ERR("[can_tx_manager]Invalid CAN TX manager configuration");
        return -ENODEV;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    if(data->frame_num == 0) {
        LOG_ERR("[can_tx_manager]No CAN frames registered for transmission");
        k_mutex_unlock(&data->lock);
        return -EINVAL;
    }
    int frame_index = -1;
    for (int i = 0; i < data->frame_num; i++) {
        if (tx_id == data->can_items[i].frame.id) {
            frame_index = i;
            break;
        }
    }

    if (frame_index < 0) {
        LOG_ERR("[can_tx_manager]Frame for tx_id 0x%03x not found", tx_id);
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }

    int ret = rp_can_tx_fillbuffer(tx_id, &data->can_items[frame_index].frame, data);
    if (ret != 0) {
        k_mutex_unlock(&data->lock);
        return ret;
    }

    /* copy the prepared frame locally, then release the lock before
     * calling can_send so the periodic thread isn't blocked by the driver
     * while the hardware transmits.  the copy prevents races on frame data.
     */
    struct can_frame tmp = data->can_items[frame_index].frame;
    k_mutex_unlock(&data->lock);
    int send_ret = can_send(cfg->can_dev, &tmp, timeout, callback, user_data);
    return send_ret;
}


static const struct can_tx_manager_api rp_can_tx_mgr_api = {
    .register_sender = rp_can_tx_manager_register,
    .unregister_sender =  rp_can_tx_manager_unregister,
    .send_frame = rp_can_tx_manager_send,
};


/* empty callback used for non‑blocking sends to keep the periodic
 * thread from being held up by the CAN API */
static void can_tx_mgr_tx_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

static K_SEM_DEFINE(s_tx_tick_sem, 0, 1);
static struct k_timer s_tx_timer;

/**
 * @brief hardware timer expiry callback (ISR context)
 *        merely gives a semaphore; avoid doing work in ISR.
 */
static void can_tx_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&s_tx_tick_sem);
}

/**
 * @brief periodic transmission thread
 *        triggered by the hardware timer at a fixed rate (see CAN_TX_MGR_TICK_MS).
 *        iterates every enabled TX manager instance and sends any due frames.
 */
static void can_tx_manager_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    static const struct device *const devs[] = {
        DT_INST_FOREACH_STATUS_OKAY(_RP_CAN_TX_MGR_DEV_PTR)
    };
    static const int dev_count = ARRAY_SIZE(devs);

    k_timer_init(&s_tx_timer, can_tx_timer_expiry, NULL);
    k_timer_start(&s_tx_timer, K_MSEC(CAN_TX_MGR_TICK_MS), K_MSEC(CAN_TX_MGR_TICK_MS));

    while (1) {
        k_sem_take(&s_tx_tick_sem, K_FOREVER);

        for (int d = 0; d < dev_count; d++) {
            const struct device *mgr = devs[d];
            const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
            rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
            if (cfg == NULL || cfg->can_dev == NULL || data == NULL) {
                continue;
            }

            k_mutex_lock(&data->lock, K_FOREVER);
            for (int f = 0; f < data->frame_num; f++) {
                rp_can_item_t *item = &data->can_items[f];

                /* frequency == 0: event-driven, skip in periodic thread */
                if (item->frequency == 0) {
                    continue;
                }

                item->tick_counter++;
                if (item->tick_counter < item->interval) {
                    continue;
                }
                item->tick_counter = 0;

                uint16_t tx_id = (uint16_t)item->frame.id;
                int ret = rp_can_tx_fillbuffer(tx_id, &item->frame, data);
                if (ret != 0) {
                    continue; /* no callback or fill failed, skip this frame */
                }
                ret = can_send(cfg->can_dev, &item->frame, K_NO_WAIT,
                               can_tx_mgr_tx_cb, NULL);
                if (ret != 0) {
                    LOG_ERR("[can_tx_manager]Periodic can_send failed for tx_id 0x%03x, err %d", tx_id, ret);
                }
            }
            k_mutex_unlock(&data->lock);
        }
    }
}

K_THREAD_DEFINE(can_tx_mgr_thread, CONFIG_CAN_TX_MANAGER_THREAD_STACK_SIZE,
                can_tx_manager_thread, NULL, NULL, NULL,
                CONFIG_CAN_TX_MANAGER_THREAD_PRIORITY, 0, 0);


#define RP_CAN_TX_MGR_DEFINE(inst)                                                              \
    static const struct rp_can_tx_cfg rp_can_tx_mgr_cfg_##inst = {                      \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                               \
    };                                                                                          \
    static struct rp_can_tx_data rp_can_tx_mgr_data_##inst;                             \
    DEVICE_DT_INST_DEFINE(inst, rp_can_tx_manager_init, NULL, &rp_can_tx_mgr_data_##inst,       \
                          &rp_can_tx_mgr_cfg_##inst, POST_KERNEL, CONFIG_CAN_TX_MANAGER_INIT_PRIORITY, &rp_can_tx_mgr_api);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_TX_MGR_DEFINE)

