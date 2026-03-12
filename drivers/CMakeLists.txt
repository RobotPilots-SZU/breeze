# Copyright (c) 2025 RobotPilots-SZU
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

add_subdirectory_ifdef(CONFIG_BUZZER buzzer)
add_subdirectory_ifdef(CONFIG_CAN_RX_MANAGER can_rx_manager)
add_subdirectory_ifdef(CONFIG_CAN_TX_MANAGER can_tx_manager)
