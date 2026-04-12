/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef CAN_TX_MANAGER_H_
#define CAN_TX_MANAGER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <stdbool.h>

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

typedef int (*can_tx_manager_api_add_frame)(const struct device *mgr,
                                            uint16_t rx_id,
                                            uint16_t tx_id,
                                            uint8_t dlc,
                                            uint8_t flags,
                                            uint16_t frequency,
                                            bool one_shot,
                                            tx_fillbuffer_cb_t fill_buffer_cb,
                                            void *user_data);

struct can_tx_manager_api
{
    can_tx_manager_api_register register_sender;
    can_tx_manager_api_unregister unregister_sender;
    can_tx_manager_api_send send_frame;
    can_tx_manager_api_add_frame add_frame;
};

/**
 * @brief Register a software TX handler inside a CAN TX manager.
 *
 * @param mgr Pointer to the CAN TX manager device
 * @param tx_id CAN identifier for outgoing frames
 * @param rx_id Reserved for future use (receive ID)
 * @param dlc Data length code for the frame (usually 8)
 * @param flags CAN frame flags (0 for standard frame)
 * @param frequency Transmit rate in Hz (0 means event-driven)
 * @param fill_buffer_cb Callback invoked to populate the frame payload
 * @param user_data Opaque pointer passed to the callback
 * @return int
 */
static inline int can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id, uint8_t dlc, uint8_t flags, uint16_t frequency, tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->register_sender == NULL) {
        return -ENOSYS;
    }
    return api->register_sender(mgr, tx_id, rx_id, dlc, flags, frequency, fill_buffer_cb, user_data);
}

/**
 * @brief Unregister a software TX handler from a CAN TX manager.
 *
 * @param mgr Pointer to the CAN TX manager device
 * @param tx_id CAN identifier for outgoing frames
 * @param rx_id Reserved for future use (receive ID)
 * @return int
 */
static inline int can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->unregister_sender == NULL) {
        return -ENOSYS;
    }
    return api->unregister_sender(mgr, tx_id, rx_id);
}

/**
 * @brief Send a CAN frame through a registered TX manager.
 *
 * @param mgr Pointer to the CAN TX manager device
 * @param timeout Timeout for the send operation
 * @param callback Callback function to be called upon completion
 * @param tx_id CAN identifier for outgoing frames
 * @param user_data Opaque pointer passed to the callback
 * @return int
 */
static inline int can_tx_manager_send(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->send_frame == NULL) {
        return -ENOSYS;
    }
    return api->send_frame(mgr, timeout, callback, tx_id, user_data);
}

/**
 * @brief Add a frame for an already registered sender(device).
 *
 * @param mgr Pointer to the CAN TX manager device
 * @param rx_id Sender/device identifier used when registering
 * @param tx_id CAN identifier for outgoing frames
 * @param dlc Data length code for the frame
 * @param flags CAN frame flags
 * @param frequency Transmit rate in Hz, ignored when one_shot=true
 * @param one_shot true: send once then auto remove; false: keep managed
 * @param fill_buffer_cb Callback invoked to populate payload before send
 * @param user_data Opaque pointer passed to callback
 * @return int
 */
static inline int can_tx_manager_add_frame(const struct device *mgr,
                                           uint16_t rx_id,
                                           uint16_t tx_id,
                                           uint8_t dlc,
                                           uint8_t flags,
                                           uint16_t frequency,
                                           bool one_shot,
                                           tx_fillbuffer_cb_t fill_buffer_cb,
                                           void *user_data)
{
    const struct can_tx_manager_api *api = (const struct can_tx_manager_api *)mgr->api;
    if(api->add_frame == NULL) {
        return -ENOSYS;
    }
    return api->add_frame(mgr, rx_id, tx_id, dlc, flags, frequency, one_shot,
                          fill_buffer_cb, user_data);
}




#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_MANAGER_H_ */
