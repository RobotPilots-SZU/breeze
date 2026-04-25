# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_library_sources(
  ./dji/can_dji.c
)

zephyr_include_directories(
  ./dji
)