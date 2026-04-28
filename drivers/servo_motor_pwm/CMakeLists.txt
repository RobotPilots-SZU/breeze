# Copyright (c) 2026 RobotPilots-SZU
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SERVO_MOTOR_PWM servo_motor_pwm.c)

