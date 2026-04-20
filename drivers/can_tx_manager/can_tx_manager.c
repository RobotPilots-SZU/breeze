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
#include <zephyr/sys/hash_map.h>
#include <drivers/can_tx_manager.h>

#include <string.h>

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(can_tx_manager);

#define CAN_TX_MGR_TICK_MS 1U
#define CAN_TX_MGR_MAX_FREQ (1000U / CAN_TX_MGR_TICK_MS)
#define RP_CAN_TX_INVALID_INDEX UINT16_MAX

#define _RP_CAN_TX_MGR_DEV_PTR(inst) DEVICE_DT_INST_GET(inst),

typedef struct rp_can_tx_cfg {
    const struct device *can_dev;
} rp_can_tx_cfg_t;

typedef struct rp_can_item {
    struct can_frame frame;
    uint16_t tx_id;
    uint16_t frequency;
    uint16_t interval;
    uint16_t tick_counter;
    bool one_shot;
    tx_fillbuffer_cb_t fill_buffer_cb;
    void *user_data;
    uint16_t next_index;
    uint16_t merge_next_index;
    uint32_t merge_epoch;
    bool merge_ready;
} rp_can_item_t;

typedef struct device_sender_cfg {
    uint16_t rx_id;
    uint16_t frame_count;
    uint16_t frames_head_index;
    uint16_t next_free_index;
    bool used;
} device_sender_cfg_t;

typedef struct rp_can_tx_data {
    struct sys_hashmap *sender_map;
    device_sender_cfg_t sender_pool[CONFIG_MAX_DEVICE_SENDERS];
    uint16_t sender_free_head;

    rp_can_item_t frame_pool[CONFIG_MAX_CAN_FRAMES];
    uint16_t frame_free_head;
    uint32_t tick_epoch;

    struct k_mutex lock;
} rp_can_tx_data_t;

/**
 * @brief 根据 rx_id 从 sender 映射中查找 sender 配置。
 *
 * @param data 管理器运行时数据。
 * @param rx_id 接收端标识。
 * @return device_sender_cfg_t* 查找到返回 sender 指针，否则返回 NULL。
 */
static device_sender_cfg_t *rp_can_tx_sender_lookup(rp_can_tx_data_t *data, uint16_t rx_id)
{
    uint64_t value = 0;

    if (!sys_hashmap_get(data->sender_map, (uint64_t)rx_id, &value)) {
        return NULL;
    }

    return (device_sender_cfg_t *)(uintptr_t)value;
}

/**
 * @brief sender_map 清理回调（当前无需对 value 做释放）。
 *
 * @param key 哈希表键。
 * @param value 哈希表值。
 * @param cookie 用户上下文。
 */
static void rp_can_tx_sender_map_clear_cb(uint64_t key, uint64_t value, void *cookie)
{
    ARG_UNUSED(key);
    ARG_UNUSED(value);
    ARG_UNUSED(cookie);
}

/**
 * @brief 内存池初始化
 *
 * @param data
 */
static void rp_can_tx_init_pools(rp_can_tx_data_t *data)
{
    /* 初始化 sender 池及其空闲链。 */
    for (uint16_t i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        data->sender_pool[i].used = false;
        data->sender_pool[i].frame_count = 0U;
        data->sender_pool[i].frames_head_index = RP_CAN_TX_INVALID_INDEX;
        data->sender_pool[i].next_free_index = (i + 1U < CONFIG_MAX_DEVICE_SENDERS) ? (i + 1U) : RP_CAN_TX_INVALID_INDEX;
    }
    data->sender_free_head = (CONFIG_MAX_DEVICE_SENDERS > 0U) ? 0U : RP_CAN_TX_INVALID_INDEX;

    /* 初始化 frame 池及其空闲链，并清空合并态。 */
    for (uint16_t i = 0; i < CONFIG_MAX_CAN_FRAMES; i++) {
        data->frame_pool[i].next_index = (i + 1U < CONFIG_MAX_CAN_FRAMES) ? (i + 1U) : RP_CAN_TX_INVALID_INDEX;
        data->frame_pool[i].merge_next_index = RP_CAN_TX_INVALID_INDEX;
        data->frame_pool[i].merge_epoch = 0U;
        data->frame_pool[i].merge_ready = false;
    }
    data->frame_free_head = (CONFIG_MAX_CAN_FRAMES > 0U) ? 0U : RP_CAN_TX_INVALID_INDEX;
}

/**
 * @brief 从 sender 池分配一个 sender 节点。
 *
 * @param data 管理器运行时数据。
 * @return device_sender_cfg_t* 成功返回节点指针，失败返回 NULL。
 */
static device_sender_cfg_t *rp_can_tx_alloc_sender(rp_can_tx_data_t *data)
{
    if (data->sender_free_head == RP_CAN_TX_INVALID_INDEX) {
        return NULL;
    }

    uint16_t idx = data->sender_free_head;
    device_sender_cfg_t *sender = &data->sender_pool[idx];
    data->sender_free_head = sender->next_free_index;

    sender->used = true;
    sender->frame_count = 0U;
    sender->frames_head_index = RP_CAN_TX_INVALID_INDEX;
    sender->next_free_index = RP_CAN_TX_INVALID_INDEX;

    return sender;
}

/**
 * @brief 将 sender 节点归还到 sender 空闲链。
 *
 * @param data 管理器运行时数据。
 * @param sender 待释放的 sender 节点。
 */
static void rp_can_tx_free_sender(rp_can_tx_data_t *data, device_sender_cfg_t *sender)
{
    uint16_t idx = (uint16_t)(sender - data->sender_pool);

    sender->used = false;
    sender->frame_count = 0U;
    sender->frames_head_index = RP_CAN_TX_INVALID_INDEX;
    sender->next_free_index = data->sender_free_head;
    data->sender_free_head = idx;
}

/**
 * @brief 从 frame 池分配一个 frame 节点。
 *
 * @param data 管理器运行时数据。
 * @param out_index 输出分配到的池下标。
 * @return rp_can_item_t* 成功返回节点指针，失败返回 NULL。
 */
static rp_can_item_t *rp_can_tx_alloc_frame(rp_can_tx_data_t *data, uint16_t *out_index)
{
    if (data->frame_free_head == RP_CAN_TX_INVALID_INDEX) {
        return NULL;
    }

    uint16_t idx = data->frame_free_head;
    rp_can_item_t *item = &data->frame_pool[idx];
    data->frame_free_head = item->next_index;

    *out_index = idx;
    return item;
}

/**
 * @brief 将 frame 节点归还到 frame 空闲链。
 *
 * @param data 管理器运行时数据。
 * @param index 待释放节点在池中的下标。
 */
static void rp_can_tx_free_frame(rp_can_tx_data_t *data, uint16_t index)
{
    rp_can_item_t *item = &data->frame_pool[index];
    item->merge_next_index = RP_CAN_TX_INVALID_INDEX;
    item->merge_epoch = 0U;
    item->merge_ready = false;
    item->next_index = data->frame_free_head;
    data->frame_free_head = index;
}

/**
 * @brief 调用 item 绑定的填充回调，更新其 frame 数据。
 *
 * @param item 需要填充的发送项。
 * @return int 回调返回值，0 表示成功。
 */
static int rp_can_tx_fillbuffer_for_item(rp_can_item_t *item)
{
    if ((item == NULL) || (item->fill_buffer_cb == NULL)) {
        return -EINVAL;
    }

    int ret = item->fill_buffer_cb(&item->frame, item->user_data);
    if (ret != 0) {
        LOG_ERR("[can_tx_manager]Fill buffer callback failed for tx_id 0x%03x, err %d",
                item->tx_id, ret);
    }

    return ret;
}

/**
 * @brief 将一个 item 的回调结果合并到目标 CAN 帧。
 *
 * @param item 当前参与合并的发送项。
 * @param merged 合并目标帧。
 * @param merged_ready 合并帧是否已初始化并可用。
 * @return bool true 表示本次合并成功，false 表示失败。
 */
static bool rp_can_tx_merge_item_into_frame(rp_can_item_t *item,
                                            struct can_frame *merged,
                                            bool *merged_ready)
{
    if ((item == NULL) || (merged == NULL) || (merged_ready == NULL)) {
        return false;
    }

    /* 首次合并时初始化目标帧头。 */
    if (!(*merged_ready)) {
        memset(merged, 0, sizeof(*merged));
        merged->id = item->tx_id;
        merged->dlc = item->frame.dlc;
        merged->flags = item->frame.flags;
    }

    /* 把当前累计结果作为回调输入，支持增量叠加。 */
    item->frame = *merged;
    int fill_ret = rp_can_tx_fillbuffer_for_item(item);
    if (fill_ret == 0) {
        /* 回调成功后回写累计结果。 */
        *merged = item->frame;
        *merged_ready = true;
        return true;
    }

    return false;
}

/**
 * @brief 在当前 tick 的合并链中查找同 tx_id 的合并槽。
 *
 * @param data 管理器运行时数据。
 * @param merge_head 合并链头索引。
 * @param tx_id 待查找的发送 ID。
 * @param epoch 当前 tick 代号。
 * @return rp_can_item_t* 找到返回槽位指针，否则返回 NULL。
 */
static rp_can_item_t *rp_can_tx_find_merge_slot(rp_can_tx_data_t *data,
                                                 uint16_t merge_head,
                                                 uint16_t tx_id,
                                                 uint32_t epoch)
{
    uint16_t idx = merge_head;
    while (idx != RP_CAN_TX_INVALID_INDEX) {
        rp_can_item_t *item = &data->frame_pool[idx];
        if (!item->merge_ready) {
            idx = item->merge_next_index;
            continue;
        }
        if (item->merge_epoch != epoch) {
            idx = item->merge_next_index;
            continue;
        }
        if (item->one_shot || (item->tx_id != tx_id)) {
            idx = item->merge_next_index;
            continue;
        }

        return item;
    }

    return NULL;
}

/**
 * @brief 构建指定 tx_id 的跨 sender 合并帧（one-shot 不参与）。
 *
 * @param data 管理器运行时数据。
 * @param tx_id 目标发送 ID。
 * @param out_frame 输出合并后的帧。
 * @return int 成功返回 0，未找到可合并项返回 -ENOENT。
 */
static int rp_can_tx_build_merged_frame(rp_can_tx_data_t *data,
                                        uint16_t tx_id,
                                        struct can_frame *out_frame)
{
    bool merged_ready = false;

    /* 遍历所有 sender，汇聚同 tx_id 的周期项。 */
    struct sys_hashmap_iterator sit = {0};
    data->sender_map->api->iter(data->sender_map, &sit);
    while (sys_hashmap_iterator_has_next(&sit)) {
        sit.next(&sit);

        device_sender_cfg_t *sender = (device_sender_cfg_t *)(uintptr_t)sit.value;
        if (sender == NULL) {
            continue;
        }

        uint16_t cur_idx = sender->frames_head_index;
        while (cur_idx != RP_CAN_TX_INVALID_INDEX) {
            rp_can_item_t *cur = &data->frame_pool[cur_idx];
            cur_idx = cur->next_index;

            if (cur->tx_id != tx_id) {
                continue;
            }
            if (cur->one_shot) {
                continue;
            }

            /* 统一走合并内核，逐项叠加到 out_frame。 */
            (void)rp_can_tx_merge_item_into_frame(cur, out_frame, &merged_ready);

        }
    }

    return merged_ready ? 0 : -ENOENT;
}

/**
 * @brief 为已注册 sender 新增一条发送项（周期或 one-shot）。
 *
 * @param mgr 管理器设备实例。
 * @param rx_id 发送项所属 sender 的 rx_id。
 * @param tx_id 发送 ID。
 * @param dlc 数据长度。
 * @param flags CAN 帧标志。
 * @param frequency 周期频率（Hz），one-shot 时忽略。
 * @param one_shot 是否为单次发送。
 * @param fill_buffer_cb 数据填充回调。
 * @param user_data 回调私有数据。
 * @return int 成功返回 0，失败返回负错误码。
 */
static int rp_can_tx_manager_add_frame_impl(const struct device *mgr, uint16_t rx_id, uint16_t tx_id,
                                            uint8_t dlc, uint8_t flags, uint16_t frequency, bool one_shot,
                                            tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    /* 参数与状态前置检查。 */
    if (!device_is_ready(mgr)) {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    rp_can_tx_data_t *data = mgr->data;
    if ((data == NULL) || (data->sender_map == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager data/map is NULL");
        return -EINVAL;
    }

    if (fill_buffer_cb == NULL) {
        LOG_ERR("[can_tx_manager]fill_buffer_cb is NULL");
        return -EINVAL;
    }

    if (!one_shot && (frequency > CAN_TX_MGR_MAX_FREQ)) {
        LOG_ERR("[can_tx_manager]Invalid frequency %d Hz (max %u Hz)",
                frequency, CAN_TX_MGR_MAX_FREQ);
        return -EINVAL;
    }

    /* 进入临界区，执行 sender/frame 池操作。 */
    k_mutex_lock(&data->lock, K_FOREVER);

    device_sender_cfg_t *sender = rp_can_tx_sender_lookup(data, rx_id);
    if (sender == NULL) {
        k_mutex_unlock(&data->lock);
        LOG_ERR("[can_tx_manager]sender(rx_id=%u) not registered", rx_id);
        return -ENOENT;
    }

    uint16_t frame_index = RP_CAN_TX_INVALID_INDEX;
    rp_can_item_t *item = rp_can_tx_alloc_frame(data, &frame_index);
    if (item == NULL) {
        k_mutex_unlock(&data->lock);
        return -ENOMEM;
    }

    /* 初始化新项的帧属性与回调上下文。 */
    memset(item, 0, sizeof(*item));
    item->tx_id = tx_id;
    item->frame.id = tx_id;
    item->frame.dlc = dlc;
    item->frame.flags = flags;
    item->one_shot = one_shot;
    item->merge_next_index = RP_CAN_TX_INVALID_INDEX;
    item->fill_buffer_cb = fill_buffer_cb;
    item->user_data = user_data;

    /* 计算周期参数：one-shot 固定为下一 tick 触发。 */
    if (one_shot) {
        item->frequency = 0U;
        item->interval = 1U;
    } else {
        item->frequency = frequency;
        if (frequency > 0U) {
            uint32_t ms_per_cycle = 1000U / frequency;
            item->interval = (uint16_t)DIV_ROUND_UP(ms_per_cycle, CAN_TX_MGR_TICK_MS);
            if (item->interval == 0U) {
                item->interval = 1U;
            }
        }
    }

    /* 头插到 sender 的 frame 链。 */
    item->next_index = sender->frames_head_index;
    sender->frames_head_index = frame_index;
    sender->frame_count++;

    k_mutex_unlock(&data->lock);
    return 0;
}

/**
 * @brief CAN TX 管理器设备初始化。
 *
 * @param dev 管理器设备实例。
 * @return int 成功返回 0，失败返回负错误码。
 */
int rp_can_tx_manager_init(const struct device *dev)
{
    const rp_can_tx_cfg_t *cfg = dev->config;
    rp_can_tx_data_t *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager init failed - invalid config or data");
        return -EINVAL;
    }

    if (data->sender_map == NULL) {
        LOG_ERR("[can_tx_manager]CAN TX manager init failed - hashmap is NULL");
        return -EINVAL;
    }

    sys_hashmap_clear(data->sender_map, rp_can_tx_sender_map_clear_cb, NULL);
    rp_can_tx_init_pools(data);

    /* 互斥锁用于保护 sender/frame 池和链表结构。 */
    k_mutex_init(&data->lock);
    return 0;
}

/**
 * @brief 注册 sender 并添加一条周期发送项。
 *
 * @param mgr 管理器设备实例。
 * @param tx_id 发送 ID。
 * @param rx_id sender 的接收端标识。
 * @param dlc 数据长度。
 * @param flags CAN 帧标志。
 * @param frequency 周期频率（Hz）。
 * @param fill_buffer_cb 数据填充回调。
 * @param user_data 回调私有数据。
 * @return int 成功返回 0，失败返回负错误码。
 */
int rp_can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id,
                               uint8_t dlc, uint8_t flags, uint16_t frequency,
                               tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    if (!device_is_ready(mgr)) {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    rp_can_tx_data_t *data = mgr->data;
    if ((data == NULL) || (data->sender_map == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager data/map is NULL");
        return -EINVAL;
    }

    /* 若 sender 不存在，则先从池中分配并写入哈希映射。 */
    k_mutex_lock(&data->lock, K_FOREVER);

    device_sender_cfg_t *sender = rp_can_tx_sender_lookup(data, rx_id);
    if (sender == NULL) {
        sender = rp_can_tx_alloc_sender(data);
        if (sender == NULL) {
            k_mutex_unlock(&data->lock);
            return -ENOMEM;
        }

        sender->rx_id = rx_id;

        int sender_ins_ret = sys_hashmap_insert(data->sender_map,
                                                (uint64_t)rx_id,
                                                (uint64_t)POINTER_TO_UINT(sender), NULL);
        if (sender_ins_ret < 0) {
            rp_can_tx_free_sender(data, sender);
            k_mutex_unlock(&data->lock);
            return sender_ins_ret;
        }
    }

    k_mutex_unlock(&data->lock);

    /* 复用统一新增逻辑，注册接口默认新增周期帧。 */
    return rp_can_tx_manager_add_frame_impl(mgr, rx_id, tx_id, dlc, flags,
                                            frequency, false,
                                            fill_buffer_cb, user_data);
}

/**
 * @brief 向已注册 sender 添加发送项。
 *
 * @param mgr 管理器设备实例。
 * @param rx_id sender 的接收端标识。
 * @param tx_id 发送 ID。
 * @param dlc 数据长度。
 * @param flags CAN 帧标志。
 * @param frequency 周期频率（Hz）。
 * @param one_shot 是否单次发送。
 * @param fill_buffer_cb 数据填充回调。
 * @param user_data 回调私有数据。
 * @return int 成功返回 0，失败返回负错误码。
 */
int rp_can_tx_manager_add_frame(const struct device *mgr,
                                uint16_t rx_id,
                                uint16_t tx_id,
                                uint8_t dlc,
                                uint8_t flags,
                                uint16_t frequency,
                                bool one_shot,
                                tx_fillbuffer_cb_t fill_buffer_cb,
                                void *user_data)
{
    return rp_can_tx_manager_add_frame_impl(mgr, rx_id, tx_id, dlc, flags,
                                            frequency, one_shot,
                                            fill_buffer_cb, user_data);
}

/**
 * @brief 从指定 sender 中注销指定 tx_id 的所有发送项。
 *
 * @param mgr 管理器设备实例。
 * @param tx_id 待注销发送 ID。
 * @param rx_id sender 的接收端标识。
 * @return int 成功返回 0，失败返回负错误码。
 */
int rp_can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id)
{
    if (!device_is_ready(mgr)) {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    rp_can_tx_data_t *data = mgr->data;
    if ((data == NULL) || (data->sender_map == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager data/map is NULL");
        return -EINVAL;
    }

    /* 遍历 sender 链表，删除所有匹配 tx_id 的节点。 */
    k_mutex_lock(&data->lock, K_FOREVER);

    device_sender_cfg_t *sender = rp_can_tx_sender_lookup(data, rx_id);
    if (sender == NULL) {
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }

    bool found = false;
    uint16_t prev_idx = RP_CAN_TX_INVALID_INDEX;
    uint16_t cur_idx = sender->frames_head_index;

    while (cur_idx != RP_CAN_TX_INVALID_INDEX) {
        rp_can_item_t *cur = &data->frame_pool[cur_idx];
        uint16_t next_idx = cur->next_index;

        if (cur->tx_id == tx_id) {
            if (prev_idx == RP_CAN_TX_INVALID_INDEX) {
                sender->frames_head_index = next_idx;
            } else {
                data->frame_pool[prev_idx].next_index = next_idx;
            }

            if (sender->frame_count > 0U) {
                sender->frame_count--;
            }

            rp_can_tx_free_frame(data, cur_idx);
            found = true;
        } else {
            prev_idx = cur_idx;
        }

        cur_idx = next_idx;
    }

    if (!found) {
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }

    /* sender 为空时回收 sender 并移除 map 键值。 */
    if (sender->frame_count == 0U) {
        (void)sys_hashmap_remove(data->sender_map, (uint64_t)rx_id, NULL);
        rp_can_tx_free_sender(data, sender);
    }

    k_mutex_unlock(&data->lock);
    return 0;
}

/**
 * @brief 立即发送指定 tx_id 的合并帧（仅合并周期项）。
 *
 * @param mgr 管理器设备实例。
 * @param timeout 发送超时。
 * @param callback 发送完成回调。
 * @param tx_id 目标发送 ID。
 * @param user_data 回调私有数据。
 * @return int 成功返回 0，失败返回负错误码。
 */
int rp_can_tx_manager_send(const struct device *mgr, k_timeout_t timeout,
                           can_tx_callback_t callback, uint16_t tx_id, void *user_data)
{
    if (mgr == NULL) {
        LOG_ERR("[can_tx_manager]CAN TX manager device is NULL");
        return -EINVAL;
    }

    const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
    rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
    if ((cfg == NULL) || (cfg->can_dev == NULL) || (data == NULL)) {
        LOG_ERR("[can_tx_manager]Invalid CAN TX manager configuration");
        return -ENODEV;
    }

    /* 在锁内构建合并帧，保证池数据一致性。 */
    k_mutex_lock(&data->lock, K_FOREVER);

    struct can_frame tmp;
    int build_ret = rp_can_tx_build_merged_frame(data, tx_id, &tmp);
    if (build_ret != 0) {
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }
    k_mutex_unlock(&data->lock);

    /* 在锁外发送，避免长时间占用临界区。 */
    return can_send(cfg->can_dev, &tmp, timeout, callback, user_data);
}

static const struct can_tx_manager_api rp_can_tx_mgr_api = {
    .register_sender = rp_can_tx_manager_register,
    .unregister_sender = rp_can_tx_manager_unregister,
    .send_frame = rp_can_tx_manager_send,
    .add_frame = rp_can_tx_manager_add_frame,
};

/**
 * @brief 周期线程使用的 CAN 发送回调（占位实现）。
 *
 * @param dev CAN 设备。
 * @param error 发送错误码。
 * @param user_data 用户上下文。
 */
static void can_tx_mgr_tx_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

static K_SEM_DEFINE(s_tx_tick_sem, 0, 1);
static struct k_timer s_tx_timer;

/**
 * @brief 定时器到期回调：释放一次发送 tick 信号。
 *
 * @param timer 定时器对象。
 */
static void can_tx_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&s_tx_tick_sem);
}

/**
 * @brief CAN TX 管理线程：处理周期帧聚合发送与 one-shot 立即发送。
 *
 * @param p1 线程参数（未使用）。
 * @param p2 线程参数（未使用）。
 * @param p3 线程参数（未使用）。
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
        /* 等待下一发送周期。 */
        k_sem_take(&s_tx_tick_sem, K_FOREVER);

        /* 每个 manager 实例独立处理。 */
        for (int d = 0; d < dev_count; d++) {
            const struct device *mgr = devs[d];
            const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
            rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
            if ((cfg == NULL) || (cfg->can_dev == NULL) || (data == NULL)) {
                continue;
            }

            k_mutex_lock(&data->lock, K_FOREVER);

            /* 递增 epoch，用于隔离每个 tick 的合并状态。 */
            data->tick_epoch++;
            if (data->tick_epoch == 0U) {
                data->tick_epoch = 1U;
            }
            uint32_t epoch = data->tick_epoch;

            struct sys_hashmap_iterator sit = {0};
            uint16_t merge_head = RP_CAN_TX_INVALID_INDEX;
            data->sender_map->api->iter(data->sender_map, &sit);
            while (sys_hashmap_iterator_has_next(&sit)) {
                sit.next(&sit);

                device_sender_cfg_t *sender = (device_sender_cfg_t *)(uintptr_t)sit.value;
                if (sender == NULL) {
                    continue;
                }

                uint16_t prev_idx = RP_CAN_TX_INVALID_INDEX;
                uint16_t cur_idx = sender->frames_head_index;
                while (cur_idx != RP_CAN_TX_INVALID_INDEX) {
                    rp_can_item_t *cur = &data->frame_pool[cur_idx];
                    uint16_t next_idx = cur->next_index;

                    /* 判断当前项在本 tick 是否到期。 */
                    bool due = false;
                    if (cur->one_shot) {
                        due = true;
                    } else if (cur->frequency > 0U) {
                        cur->tick_counter++;
                        if (cur->tick_counter >= cur->interval) {
                            cur->tick_counter = 0U;
                            due = true;
                        }
                    }

                    if (!due) {
                        prev_idx = cur_idx;
                        cur_idx = next_idx;
                        continue;
                    }

                    /* one-shot：立即发送并从 sender 链移除。 */
                    if (cur->one_shot) {
                        int fill_ret = rp_can_tx_fillbuffer_for_item(cur);
                        if (fill_ret == 0) {
                            int send_ret = can_send(cfg->can_dev, &cur->frame, K_NO_WAIT,
                                                    can_tx_mgr_tx_cb, NULL);
                            if (send_ret != 0) {
                                LOG_ERR("[can_tx_manager]can_send failed for tx_id 0x%03x, err %d",
                                        cur->tx_id, send_ret);
                            }
                        }

                        if (prev_idx == RP_CAN_TX_INVALID_INDEX) {
                            sender->frames_head_index = next_idx;
                        } else {
                            data->frame_pool[prev_idx].next_index = next_idx;
                        }

                        if (sender->frame_count > 0U) {
                            sender->frame_count--;
                        }

                        rp_can_tx_free_frame(data, cur_idx);

                        if (sender->frame_count == 0U) {
                            uint16_t sender_rx_id = sender->rx_id;
                            (void)sys_hashmap_remove(data->sender_map, (uint64_t)sender_rx_id, NULL);
                            rp_can_tx_free_sender(data, sender);
                        }

                        cur_idx = next_idx;
                        continue;
                    }

                    /* periodic：按 tx_id 聚合到本 tick 的 merge 槽。 */
                    rp_can_item_t *merge_slot = rp_can_tx_find_merge_slot(data, merge_head, cur->tx_id, epoch);
                    if (merge_slot == NULL) {
                        /* 当前 tx_id 首次出现，创建主槽并加入 merge 链。 */
                        cur->merge_epoch = epoch;
                        cur->merge_ready = false;
                        cur->merge_next_index = merge_head;
                        merge_head = cur_idx;
                        (void)rp_can_tx_merge_item_into_frame(cur, &cur->frame, &cur->merge_ready);
                    } else {
                        /* 已有主槽，继续叠加数据。 */
                        (void)rp_can_tx_merge_item_into_frame(cur,
                                                              &merge_slot->frame,
                                                              &merge_slot->merge_ready);
                    }

                    prev_idx = cur_idx;
                    cur_idx = next_idx;
                }
            }

            /* 发送本 tick 所有聚合完成的 periodic 槽。 */
            uint16_t idx = merge_head;
            while (idx != RP_CAN_TX_INVALID_INDEX) {
                rp_can_item_t *item = &data->frame_pool[idx];
                uint16_t next_merge_idx = item->merge_next_index;

                if (item->merge_ready && (item->merge_epoch == epoch) && !item->one_shot) {
                    int send_ret = can_send(cfg->can_dev, &item->frame, K_NO_WAIT,
                                            can_tx_mgr_tx_cb, NULL);
                    if (send_ret != 0) {
                        LOG_ERR("[can_tx_manager]can_send failed for tx_id 0x%03x, err %d",
                                item->tx_id, send_ret);
                    }
                }

                /* 清理临时合并状态，避免跨 tick 污染。 */
                item->merge_ready = false;
                item->merge_next_index = RP_CAN_TX_INVALID_INDEX;
                idx = next_merge_idx;
            }

            k_mutex_unlock(&data->lock);
        }
    }
}

K_THREAD_DEFINE(can_tx_mgr_thread, CONFIG_CAN_TX_MANAGER_THREAD_STACK_SIZE,
                can_tx_manager_thread, NULL, NULL, NULL,
                CONFIG_CAN_TX_MANAGER_THREAD_PRIORITY, 0, 0);

#define RP_CAN_TX_MGR_MAP_DEFINE(inst) \
    SYS_HASHMAP_DEFINE(rp_can_tx_sender_map_##inst);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_TX_MGR_MAP_DEFINE)

#define RP_CAN_TX_MGR_DEFINE(inst)                                                          \
    static const struct rp_can_tx_cfg rp_can_tx_mgr_cfg_##inst = {                         \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                          \
    };                                                                                       \
    static struct rp_can_tx_data rp_can_tx_mgr_data_##inst = {                              \
        .sender_map = &rp_can_tx_sender_map_##inst,                                          \
    };                                                                                       \
    DEVICE_DT_INST_DEFINE(inst, rp_can_tx_manager_init, NULL, &rp_can_tx_mgr_data_##inst,  \
                          &rp_can_tx_mgr_cfg_##inst, POST_KERNEL,                           \
                          CONFIG_CAN_TX_MANAGER_INIT_PRIORITY, &rp_can_tx_mgr_api);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_TX_MGR_DEFINE)
