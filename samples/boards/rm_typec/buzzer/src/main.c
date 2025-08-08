#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define BUZZER_PERIOD_USEC 1000  // 1kHz tone
#define BUZZER_DURATION_MS 500   // Buzz duration

// Get PWM device and channel from buzzer0 alias
#define BUZZER_NODE DT_ALIAS(buzzer0)
#define BUZZER_PWM_CTLR DT_PWMS_CTLR(BUZZER_NODE)
#define BUZZER_PWM_CHANNEL DT_PWMS_CHANNEL(BUZZER_NODE)

// Compile-time check for buzzer0 alias existence
#if !DT_NODE_EXISTS(BUZZER_NODE)
#error "buzzer0 alias is not defined in device tree"
#endif

int main(void)
{
    const struct device *buzzer_dev = DEVICE_DT_GET(BUZZER_PWM_CTLR);

    if (!device_is_ready(buzzer_dev)) {
        printk("Error: PWM device not ready\n");
        return -1;
    }

    printk("Buzzer test started (PWM channel %d)\n", BUZZER_PWM_CHANNEL);

    while (1) {
        // Turn on the buzzer with 50% duty cycle
        pwm_set(buzzer_dev, BUZZER_PWM_CHANNEL, PWM_USEC(BUZZER_PERIOD_USEC), PWM_USEC(BUZZER_PERIOD_USEC / 2), 0);
        k_msleep(BUZZER_DURATION_MS);

        // Turn off the buzzer
        pwm_set(buzzer_dev, BUZZER_PWM_CHANNEL, PWM_USEC(BUZZER_PERIOD_USEC), 0, 0);
        k_msleep(500);
    }
	return 0;
}
