cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED BOARD)
	set(BOARD damiao_mc02)
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_sample)

target_sources(app PRIVATE src/main.cpp)
