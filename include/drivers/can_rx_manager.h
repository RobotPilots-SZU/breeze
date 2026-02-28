/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef CAN_RX_MANAGER_H_
#define CAN_RX_MANAGER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*can_rx_handler_t)(const struct can_frame *frame, void *user_data);

/**
 * @brief Register a software RX handler inside a CAN RX manager.
 * @param mgr      CAN RX manager device.
 * @param filter   CAN filter used for software matching (standard/extended is controlled by flags).
 * @param handler  Called in manager RX thread context.
 * @param user_data Opaque pointer passed to handler.
 *
 * @retval >=0 Listener ID (can be used for unregister).
 */
typedef int (*can_rx_manager_api_register)(const struct device *mgr, const struct can_filter *filter,
                                           can_rx_handler_t handler, void *user_data);

/**
 * @brief Unregister a previously registered listener.
 *
 * @param mgr CAN RX manager device.
 * @param listener_id Listener ID returned by can_rx_manager_api_register().
 * @retval 0 on success.
 */
typedef int (*can_rx_manager_api_unregister)(const struct device *mgr, int listener_id);

/**
 * @brief Calculate current CAN bus load (percentage) over the interval since last call.
 * @param mgr CAN RX manager device
 * @param nominal_bitrate_bps Bitrate used for arbitration/CRC/control (bps)
 * @param data_bitrate_bps Bitrate used for FD data phase when BRS is set (bps). If 0, FD data is assumed to use nominal rate.
 * @return load percentage in range [0.0 .. 100.0], negative on error.
 */
typedef float (*can_rx_manager_api_calculate_load)(const struct device *mgr, uint32_t nominal_bitrate_bps, uint32_t data_bitrate_bps);


struct can_rx_manager_api
{
    can_rx_manager_api_register register_listener;
    can_rx_manager_api_unregister unregister_listener;
    can_rx_manager_api_calculate_load calculate_load;
};

/**
 * @brief Register a software RX handler inside a CAN RX manager.
 * @param mgr      CAN RX manager device.
 * @param filter   CAN filter used for software matching (standard/extended is controlled by flags).
 * @param handler  Called in manager RX thread context.
 * @param user_data Opaque pointer passed to handler (should be the CAN device data pointer).
 * @return int
 */
static inline int can_rx_manager_register(const struct device *mgr, const struct can_filter *filter,
                                        can_rx_handler_t handler, void *user_data)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->register_listener == NULL) {
        return -ENOSYS;
    }
    return api->register_listener(mgr, filter, handler, user_data);
}

/**
 * @brief Unregister a previously registered listener.
 *
 * @param mgr CAN RX manager device.
 * @param listener_id Listener ID returned by can_rx_manager_register().
 * @return int 0 on success, negative error code on failure.
 */
static inline int can_rx_manager_unregister(const struct device *mgr, int listener_id)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->unregister_listener == NULL) {
        return -ENOSYS;
    }
    return api->unregister_listener(mgr, listener_id);
}

/**
 * @brief Calculate the CAN bus load percentage over the interval since the last call.
 *
 * @param mgr                CAN RX manager device
 * @param nominal_bitrate_bps Nominal (arbitration) bitrate in bps
 * @param data_bitrate_bps   FD data-phase bitrate in bps; 0 means use nominal rate (classic CAN)
 * @return float             Load percentage [0.0 .. 100.0], negative value on error
 */
static inline float can_rx_manager_calculate_load(const struct device *mgr, uint32_t nominal_bitrate_bps, uint32_t data_bitrate_bps)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->calculate_load == NULL) {
        return -ENOSYS;
    }
    return api->calculate_load(mgr, nominal_bitrate_bps, data_bitrate_bps);
}


#ifdef __cplusplus
}
#endif

#endif /* CAN_RX_MANAGER_H_ */
