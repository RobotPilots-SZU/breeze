/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef CAN_TX_MANAGER_H_
#define CAN_TX_MANAGER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*tx_fillbuffer_cb_t)(struct can_frame *frame, void *user_data);

/**
  * @brief Register a software TX handler inside a CAN TX manager.
 */
typedef int (*can_tx_manager_api_register)(const struct device *mgr, uint16_t tx_id, uint16_t rx_id, uint8_t dlc, uint8_t flags, uint16_t frequency, tx_fillbuffer_cb_t fill_buffer_cb, void *user_data);

typedef int (*can_tx_manager_api_unregister)(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id);

typedef int (*can_tx_manager_api_send)(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data);

struct can_tx_manager_api
{
    can_tx_manager_api_register register_sender;
    can_tx_manager_api_unregister unregister_sender;
    can_tx_manager_api_send send_frame;
};

static inline int can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id, uint8_t dlc, uint8_t flags, uint16_t frequency, tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->register_sender == NULL) {
        return -ENOSYS;
    }
    return api->register_sender(mgr, tx_id, rx_id, dlc, flags, frequency, fill_buffer_cb, user_data);
}

static inline int can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->unregister_sender == NULL) {
        return -ENOSYS;
    }
    return api->unregister_sender(mgr, tx_id, rx_id);
}

static inline int can_tx_manager_send(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->send_frame == NULL) {
        return -ENOSYS;
    }
    return api->send_frame(mgr, timeout, callback, tx_id, user_data);
}




#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_MANAGER_H_ */
