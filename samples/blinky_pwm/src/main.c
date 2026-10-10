/*
 * Copyright (c) 2020 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define LED_PWM_NODE_ID	 DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)

const char *led_label[] = {
    DT_FOREACH_CHILD_SEP_VARGS(LED_PWM_NODE_ID, DT_PROP_OR, (,), label, NULL)
};

const int num_leds = ARRAY_SIZE(led_label);

#define MAX_BRIGHTNESS	100

/**
 * @brief Run tests on a single LED using the LED API syscalls.
 *
 * @param led_pwm LED PWM device.
 * @param led Number of the LED to test.
 */
static void run_led_test(const struct device *led_pwm, uint8_t led)
{
    int err;
    int16_t level;

    LOG_INF("Testing LED %d - %s", led, led_label[led] ? : "no label");

    /* Turn LED on. */
    err = led_on(led_pwm, led);
    if (err < 0) {
        LOG_ERR("err=%d", err);
        return;
    }
    LOG_INF("  Turned on");
    k_sleep(K_MSEC(1000));

    /* Turn LED off. */
    err = led_off(led_pwm, led);
    if (err < 0) {
        LOG_ERR("err=%d", err);
        return;
    }
    LOG_INF("  Turned off");
    k_sleep(K_MSEC(1000));

    /* Increase LED brightness gradually up to the maximum level. */
    LOG_INF("  Increasing brightness gradually");
    for (level = 0; level <= MAX_BRIGHTNESS; level++) {
        err = led_set_brightness(led_pwm, led, level);
        if (err < 0) {
            LOG_ERR("err=%d brightness=%d\n", err, level);
            return;
        }
        k_sleep(K_MSEC(CONFIG_FADE_DELAY));
    }
    k_sleep(K_MSEC(1000));

    /* Decrease LED brightness gradually down to the minimum level. */
    LOG_INF("  Decreasing brightness gradually");
    for (level = MAX_BRIGHTNESS; level >= 0; level--) {
        err = led_set_brightness(led_pwm, led, level);
        if (err < 0) {
            LOG_ERR("err=%d brightness=%d\n", err, level);
            return;
        }
        k_sleep(K_MSEC(CONFIG_FADE_DELAY));
    }
    k_sleep(K_MSEC(1000));

#if CONFIG_BLINK_DELAY_SHORT > 0
    /* Start LED blinking (short cycle) */
    err = led_blink(led_pwm, led, CONFIG_BLINK_DELAY_SHORT, CONFIG_BLINK_DELAY_SHORT);
    if (err < 0) {
        LOG_ERR("err=%d", err);
        return;
    }
    LOG_INF("  Blinking "
        "on: " STRINGIFY(CONFIG_BLINK_DELAY_SHORT) " msec, "
        "off: " STRINGIFY(CONFIG_BLINK_DELAY_SHORT) " msec");
    k_sleep(K_MSEC(5000));
#endif

#if CONFIG_BLINK_DELAY_LONG > 0
    /* Start LED blinking (long cycle) */
    err = led_blink(led_pwm, led, CONFIG_BLINK_DELAY_LONG, CONFIG_BLINK_DELAY_LONG);
    if (err < 0) {
        LOG_ERR("err=%d", err);
        LOG_INF("  Cycle period not supported - "
            "on: " STRINGIFY(CONFIG_BLINK_DELAY_LONG) "  msec, "
            "off: " STRINGIFY(CONFIG_BLINK_DELAY_LONG) " msec");
    } else {
        LOG_INF("  Blinking "
            "on: " STRINGIFY(CONFIG_BLINK_DELAY_LONG) " msec, "
            "off: " STRINGIFY(CONFIG_BLINK_DELAY_LONG) " msec");
    }
    k_sleep(K_MSEC(5000));
#endif

    /* Turn LED off. */
    err = led_off(led_pwm, led);
    if (err < 0) {
        LOG_ERR("err=%d", err);
        return;
    }
    LOG_INF("  Turned off, loop end");
}

/**
 * @brief Run RGB fade test on multiple LEDs to create color transitions.
 *
 * @param led_pwm LED PWM device.
 */
static void run_rgb_fade_test(const struct device *led_pwm)
{
    int err;
    uint16_t step;
    const uint16_t total_steps = 360; /* Full color wheel */
    const uint16_t brightness_steps = 100; /* Brightness fade steps */

    if (num_leds < 3) {
        LOG_INF("RGB fade test requires at least 3 LEDs, only %d available", num_leds);
        return;
    }

    LOG_INF("Starting smooth RGB fade test using first 3 LEDs");

    /* Brightness fade in */
    LOG_INF("  Brightness fade in");
    for (uint16_t bright_step = 0; bright_step <= brightness_steps; bright_step++) {
        uint8_t brightness = (bright_step * MAX_BRIGHTNESS) / brightness_steps;
        
        /* Start with red color */
        err = led_set_brightness(led_pwm, 0, brightness);   /* Red LED */
        if (err < 0) LOG_ERR("Red LED err=%d", err);
        
        err = led_set_brightness(led_pwm, 1, 0);            /* Green LED */
        if (err < 0) LOG_ERR("Green LED err=%d", err);
        
        err = led_set_brightness(led_pwm, 2, 0);            /* Blue LED */
        if (err < 0) LOG_ERR("Blue LED err=%d", err);
        
        k_sleep(K_MSEC(20));
    }

    /* Smooth color wheel transition */
    LOG_INF("  Color wheel transition");
    for (step = 0; step < total_steps; step++) {
        float angle = (float)step * 360.0f / total_steps;
        uint8_t red_brightness = 0, green_brightness = 0, blue_brightness = 0;

        /* HSV to RGB conversion for smooth color transitions */
        if (angle < 60) {
            /* Red to Yellow */
            red_brightness = MAX_BRIGHTNESS;
            green_brightness = (uint8_t)(MAX_BRIGHTNESS * angle / 60.0f);
            blue_brightness = 0;
        } else if (angle < 120) {
            /* Yellow to Green */
            red_brightness = (uint8_t)(MAX_BRIGHTNESS * (120 - angle) / 60.0f);
            green_brightness = MAX_BRIGHTNESS;
            blue_brightness = 0;
        } else if (angle < 180) {
            /* Green to Cyan */
            red_brightness = 0;
            green_brightness = MAX_BRIGHTNESS;
            blue_brightness = (uint8_t)(MAX_BRIGHTNESS * (angle - 120) / 60.0f);
        } else if (angle < 240) {
            /* Cyan to Blue */
            red_brightness = 0;
            green_brightness = (uint8_t)(MAX_BRIGHTNESS * (240 - angle) / 60.0f);
            blue_brightness = MAX_BRIGHTNESS;
        } else if (angle < 300) {
            /* Blue to Magenta */
            red_brightness = (uint8_t)(MAX_BRIGHTNESS * (angle - 240) / 60.0f);
            green_brightness = 0;
            blue_brightness = MAX_BRIGHTNESS;
        } else {
            /* Magenta to Red */
            red_brightness = MAX_BRIGHTNESS;
            green_brightness = 0;
            blue_brightness = (uint8_t)(MAX_BRIGHTNESS * (360 - angle) / 60.0f);
        }

        /* Set brightness for RGB LEDs */
        err = led_set_brightness(led_pwm, 0, red_brightness);   /* Red LED */
        if (err < 0) LOG_ERR("Red LED err=%d", err);

        err = led_set_brightness(led_pwm, 1, green_brightness); /* Green LED */
        if (err < 0) LOG_ERR("Green LED err=%d", err);

        err = led_set_brightness(led_pwm, 2, blue_brightness);  /* Blue LED */
        if (err < 0) LOG_ERR("Blue LED err=%d", err);

        k_sleep(K_MSEC(30)); /* Smooth transition delay */
    }

    /* Brightness fade out */
    LOG_INF("  Brightness fade out");
    for (uint16_t bright_step = brightness_steps; bright_step > 0; bright_step--) {
        uint8_t brightness = (bright_step * MAX_BRIGHTNESS) / brightness_steps;
        
        /* End with red color */
        err = led_set_brightness(led_pwm, 0, brightness);   /* Red LED */
        if (err < 0) LOG_ERR("Red LED err=%d", err);
        
        err = led_set_brightness(led_pwm, 1, 0);            /* Green LED */
        if (err < 0) LOG_ERR("Green LED err=%d", err);
        
        err = led_set_brightness(led_pwm, 2, 0);            /* Blue LED */
        if (err < 0) LOG_ERR("Blue LED err=%d", err);
        
        k_sleep(K_MSEC(20));
    }

    /* Turn off all RGB LEDs */
    for (uint8_t i = 0; i < 3; i++) {
        err = led_off(led_pwm, i);
        if (err < 0) {
            LOG_ERR("Failed to turn off LED %d, err=%d", i, err);
        }
    }
    LOG_INF("RGB fade test completed");
}

int main(void)
{
    const struct device *led_pwm;
    uint8_t led;

    led_pwm = DEVICE_DT_GET(LED_PWM_NODE_ID);
    if (!device_is_ready(led_pwm)) {
        LOG_ERR("Device %s is not ready", led_pwm->name);
        return 0;
    }

    if (!num_leds) {
        LOG_ERR("No LEDs found for %s", led_pwm->name);
        return 0;
    }

    do {
        for (led = 0; led < num_leds; led++) {
            run_led_test(led_pwm, led);
        }
        
        /* Run RGB fade test after individual LED tests */
        run_rgb_fade_test(led_pwm);
        
        k_sleep(K_MSEC(2000)); /* Pause between full cycles */
    } while (true);
    return 0;
}
