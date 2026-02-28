cmake_minimum_required(VERSION 3.20)

set(BOARD damiao_mc02)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can_rx_manage)

list(APPEND DTC_BINDINGS_DIRS
    ${bindings_DIR}/can
)

target_sources(app PRIVATE
    src/main.c
    ${self_driver_DIR}/can_rx_manager/can_rx_manager.c
)
target_include_directories(app PRIVATE include)

target_include_directories(app PRIVATE
    include
    ${self_driver_DIR}/can_rx_manager
)
