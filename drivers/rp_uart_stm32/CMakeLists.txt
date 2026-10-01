# Copyright (c) 2026 RobotPilots
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RP_UART_STM32 rp_uart_stm32.c)
