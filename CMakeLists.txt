# Copyright (c) 2025 RobotPilots-SZU
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).


zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)
