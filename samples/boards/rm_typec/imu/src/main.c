#include "zephyr/device.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
int main()
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
	// const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
	struct sensor_value acc[3], gyr[3];
	
	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 0;
	}
	printk("Device %p name is %s\n", dev, dev->name);
	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		// sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
		printk("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d\n", acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2);
		printk("\n");
		k_msleep(500);
	}
}
