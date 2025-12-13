# Copyright (c) 2025 RobotPilots-SZU
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).


zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

set(BREEZE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Breeze root directory")
set(bindings_DIR ${BREEZE_ROOT}/dts/bindings CACHE PATH "DTS bindings directory")
set(self_driver_DIR ${BREEZE_ROOT}/drivers CACHE PATH "Drivers directory")


add_subdirectory(drivers)
add_subdirectory(lib)
