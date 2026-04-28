// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/logging/log.h>
// #include <drivers/servo_motor_pwm.h>

// /* 
// *   日志模块注册
// *   改成LOG_LEVEL_DBG即可输出DBG级别的日志
// *   输出DBG级别日志需要注意 CONFIG_LOG_PROCESS_THREAD_STACK_SIZE 配置，防止日志过多导致栈溢出
// */
// LOG_MODULE_REGISTER(servo_motor, LOG_LEVEL_INF);

// static const struct device *servo = DEVICE_DT_GET(DT_ALIAS(servo0));
// static const struct pwm_dt_spec servo_pwm = PWM_DT_SPEC_GET(DT_ALIAS(servo0));

// int main(void)
// {
// 	if (!device_is_ready(servo_pwm.dev)) {
// 		LOG_ERR("Servo PWM device is not ready");
// 		return 0;
// 	}

// 	if (servo_set_mechanical_limit_angle(servo, 180.0f, 0.0f) != 0) {
// 		LOG_ERR("Failed to set mechanical limits, %s", servo->name);
// 		return 0;
// 	}

// 	if (servo_set_offset_angle(servo, 0.0f) != 0) {
// 		LOG_ERR("Failed to set offset, %s", servo->name);
// 		return 0;
// 	}

// 	/* Get PWM controller/channel/period from the servo node pwms property. */
// 	while (1) 
// 	{
// 		int ret;
// 		for (int pulse = 100000; pulse <= 550000; pulse += 10000) {
// 			ret = pwm_set_pulse_dt(&servo_pwm, pulse);
// 			LOG_INF("set pulse %d ns, ret=%d", pulse, ret);
// 			k_msleep(1000);
// 		}

// 		for (int pulse = 3000000; pulse >= 2400000; pulse -= 10000) {
// 			ret = pwm_set_pulse_dt(&servo_pwm, pulse);
// 			LOG_INF("set pulse %d ns, ret=%d", pulse, ret);
// 			k_msleep(1000);
// 		}
// 	}

// 	return 0;
// }

