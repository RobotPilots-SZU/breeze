# Copyright (c) 2026 RobotPilots
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_UART_STM32_FIX uart_stm32_fix.c)
