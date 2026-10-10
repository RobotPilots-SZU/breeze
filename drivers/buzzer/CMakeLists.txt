# SPDX-License-Identifier: Apache-2.0

zephyr_library()


zephyr_library_sources_ifdef(CONFIG_MLT5020_BUZZER_PWM MLT5020_pwm.c)

