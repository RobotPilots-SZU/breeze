# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

# 把官方 bmi08x 头文件目录加入 include 路径
target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PRIVATE
    ${ZEPHYR_BASE}/drivers/sensor/bosch/bmi08x
)

if(CONFIG_RP_BMI08X_STREAM)
    # 获取官方库的源文件列表
    get_target_property(BMI08X_SRCS ${ZEPHYR_CURRENT_LIBRARY} SOURCES)
    
    if(BMI08X_SRCS)
        # 只在官方启用了对应流模式时才剔除官方文件
        if(CONFIG_BMI08X_ACCEL_STREAM)
            list(FILTER BMI08X_SRCS EXCLUDE REGEX "bmi08x_accel_stream\\.c$")
            list(APPEND BMI08X_SRCS
                ${CMAKE_CURRENT_SOURCE_DIR}/bmi08x_accel_stream.c)
        endif()

        if(CONFIG_BMI08X_GYRO_STREAM)
            list(FILTER BMI08X_SRCS EXCLUDE REGEX "bmi08x_gyro_stream\\.c$")
            list(APPEND BMI08X_SRCS
                ${CMAKE_CURRENT_SOURCE_DIR}/bmi08x_gyro_stream.c)
        endif()

        set_target_properties(${ZEPHYR_CURRENT_LIBRARY} PROPERTIES SOURCES "${BMI08X_SRCS}")
    
    endif()
endif()
