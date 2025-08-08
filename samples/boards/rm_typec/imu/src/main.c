#include "zephyr/device.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_sample, LOG_LEVEL_INF);

int main()
{
	const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
	const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
	struct sensor_value acc[3], gyr[3];
	
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device %s is not ready", accel_dev->name);
		return 0;
	}
	
	if (!device_is_ready(gyro_dev)) {
		LOG_ERR("Gyroscope device %s is not ready", gyro_dev->name);
		return 0;
	}
	
	LOG_INF("Accelerometer device %p name is %s", accel_dev, accel_dev->name);
	LOG_INF("Gyroscope device %p name is %s", gyro_dev, gyro_dev->name);
	
	while (1) {
		// 读取加速度计数据
		sensor_sample_fetch(accel_dev);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		
		// 读取陀螺仪数据
		sensor_sample_fetch(gyro_dev);
		sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
		
		LOG_INF("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d | GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d", 
		       acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2, gyr[1].val1, gyr[1].val2, gyr[2].val1, gyr[2].val2);
		
		k_msleep(500);
	}
}