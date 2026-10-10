cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(buzzer)

# 驱动源由 module 层（drivers/buzzer/CMakeLists.txt）添加，这里不要再手动加一遍，
# 否则同一个 .c 会进 app 和 zephyr 两个库 → 重复定义 log_const_buzzer_pwm / __device_dts_ord_*
target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE include)
