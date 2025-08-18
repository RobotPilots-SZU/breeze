/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#define GEAR_COUNT 8
#define STEP ((max_pulse - min_pulse) / (GEAR_COUNT - 1))

// 提取舵机的 pwm_dt_spec
#define GET_SERVO_PWM_SPEC(node_id) \
    PWM_DT_SPEC_GET(node_id),

// 获取父节点路径
#define SERVO_PWM_NODE_ID DT_PATH(pwmservo)

// 动态生成舵机数组
static const struct pwm_dt_spec servos[] = {
    DT_FOREACH_CHILD(SERVO_PWM_NODE_ID, GET_SERVO_PWM_SPEC)
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);

#define NUM_SERVOS ARRAY_SIZE(servos)

static const uint32_t min_pulse = PWM_MSEC(0.5);
static const uint32_t max_pulse = PWM_MSEC(2.5);

int main(void)
{
	uint32_t pulse_width = min_pulse;
	int current_gear = 0;
	int ret;
	bool button_pressed = false;
	bool last_button_state = false;

	printk("Servomotor control with %d gears, %d servos detected\n", GEAR_COUNT, NUM_SERVOS);

	// Check all servos are ready
	for (int i = 0; i < NUM_SERVOS; i++) {
		if (!pwm_is_ready_dt(&servos[i])) {
			printk("Warning: Servo %d PWM device is not ready\n", i);
		} else {
			printk("Servo %d ready\n", i);
		}
	}

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: GPIO device %s is not ready\n", button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure button GPIO\n", ret);
		return 0;
	}

	// Set initial position for all servos
	for (int i = 0; i < NUM_SERVOS; i++) {
		if (pwm_is_ready_dt(&servos[i])) {
			ret = pwm_set_pulse_dt(&servos[i], pulse_width);
			if (ret < 0) {
				printk("Error %d: failed to set initial pulse width for servo %d\n", ret, i);
				return 0;
			}
		}
	}

	printk("All servos, Gear %d: pulse width %d us\n", current_gear + 1, pulse_width);

	while (1) {
		button_pressed = !gpio_pin_get_dt(&button); // Button is active low

		// Detect button press (edge detection)
		if (button_pressed && !last_button_state) {
			current_gear = (current_gear + 1) % GEAR_COUNT;
			pulse_width = min_pulse + (current_gear * STEP);

			// Update all servos
			for (int i = 0; i < NUM_SERVOS; i++) {
				if (pwm_is_ready_dt(&servos[i])) {
					ret = pwm_set_pulse_dt(&servos[i], pulse_width);
					if (ret < 0) {
						printk("Error %d: failed to set pulse width for servo %d\n", ret, i);
						return 0;
					}
				}
			}

			printk("All servos, Gear %d: pulse width %d us\n", current_gear + 1, pulse_width);
		}

		last_button_state = button_pressed;
		k_sleep(K_MSEC(50)); // Debounce delay
	}
	return 0;
}
