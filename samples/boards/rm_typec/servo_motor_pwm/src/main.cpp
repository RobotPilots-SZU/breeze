#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <drivers/servo_motor_pwm.h>

/* 
*   日志模块注册
*   改成LOG_LEVEL_DBG即可输出DBG级别的日志
*   输出DBG级别日志需要注意 CONFIG_LOG_PROCESS_THREAD_STACK_SIZE 配置，防止日志过多导致栈溢出
*/
LOG_MODULE_REGISTER(servo_motor, LOG_LEVEL_INF);

static const struct device *servo = DEVICE_DT_GET(DT_ALIAS(servo0));

int main(void)
{

	if (!device_is_ready(servo)) {
		LOG_ERR("Servo device %s is not ready", servo->name);
		return 0;
	}

	if (servo_set_mechanical_limit_angle(servo, 180.0f, 0.0f) != 0) {
		LOG_ERR("Failed to set mechanical limits, %s", servo->name);
		return 0;
	}

	if (servo_set_offset_angle(servo, 0.0f) != 0) {
		LOG_ERR("Failed to set offset, %s", servo->name);
		return 0;
	}

	while (1) 
	{
		int ret;
		for (int angle = 0; angle <= 180; angle += 10) {
			ret = servo_set_angle(servo, (float)angle);
			// ret = servo_rotate_angle(servo, 1, (float)angle);
			LOG_DBG("set angle %d, ret=%d", angle, ret);
			k_msleep(1000);
		}

		for (int angle = 180; angle >= 0; angle -= 10) {
			ret = servo_set_angle(servo, (float)angle);
			// ret = servo_rotate_angle(servo, -1, (float)angle);
			LOG_DBG("set angle %d, ret=%d", angle, ret);
			k_msleep(1000);
		}

	}

	return 0;
}

