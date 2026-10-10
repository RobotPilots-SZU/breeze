#include "zephyr/device.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

LOG_MODULE_REGISTER(imu_sample, LOG_LEVEL_INF); // 开启日志模块
// 获取设备树中加速度计和陀螺仪的设备结构体指针
const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
// 定义触发模式为当传感器数据到达水印即一定数量，要在设备树中定义，触发时包含传感器数据
#define STREAM_TRIGGERS                                        \
	{SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE }
// 创建 RTIO I/O 设备实例，关联到加速度计设备节点
SENSOR_DT_STREAM_IODEV(accel_iodev, DT_NODELABEL(bmi08x_accel), STREAM_TRIGGERS);
SENSOR_DT_STREAM_IODEV(gyro_iodev, DT_NODELABEL(bmi08x_gyro), STREAM_TRIGGERS);
//内存池定义，提供给异步读取使用
RTIO_DEFINE_WITH_MEMPOOL(ctx, 4, 4, 20, 256, sizeof(void *));
/* 参数说明：名称, sq大小, cq大小, 内存池块数, 每块大小, 对齐方式 */

typedef struct
{
	float raw_acc_x;
	float raw_acc_y;
	float raw_acc_z;
	float raw_gyr_x;
	float raw_gyr_y;
	float raw_gyr_z;

} raw_info_t;

typedef struct
{
	float yaw;
	float pitch;
	float roll;

	float rate_yaw;
	float rate_pitch;
	float rate_roll;
} base_info_t;

typedef enum
{
	DEV_ONLINE,
	DEV_OFFLINE,
} dev_status_e;

typedef struct
{
	dev_status_e dev_status;
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;

} state_info_t;

typedef struct
{
	raw_info_t raw_info;
	base_info_t base_info;
	state_info_t state_info;
	int16_t temp;
} imu_sensor_t;

imu_sensor_t imu_sensor = {
	.raw_info = {0},
	.base_info = {0},
	.state_info = {
		.dev_status = DEV_OFFLINE,
		.offline_cnt = 0,
		.offline_max_cnt = 10, // 可以在这里设置默认值
	},
	.temp = 0,
};
/*
*将数据定点数转化成浮点数
*@param q 定点数值
*@param shift 定点数的小数位数
*@return 转换后的浮点数值
*/
static float sensor_qvalue_to_float(int32_t q , uint8_t shift)
{
	uint32_t int_part = __PRIq_arg_get_int(q, shift);
	uint32_t frac_part = __PRIq_arg_get_frac(q, 6, shift);
	float value = (float)int_part + (float)frac_part / 1000000.0f;
	if (q < 0)
	{
		value = -value;	
	}
	return value;
};


static void imu_init(imu_sensor_t *imu)
{
	imu->state_info.dev_status = DEV_OFFLINE;

	if (!device_is_ready(accel_dev))
	{
		LOG_ERR("Accelerometer device %s is not ready", accel_dev->name);
		imu->state_info.dev_status = DEV_OFFLINE;
		return;
	}

	if (!device_is_ready(gyro_dev))
	{
		LOG_ERR("Gyroscope device %s is not ready", gyro_dev->name);
		imu->state_info.dev_status = DEV_OFFLINE;
		return;
	}
	/* 验证解码器支持 */
	const struct sensor_decoder_api *decoder;
	if (sensor_get_decoder(accel_dev, &decoder) < 0)
	{
		LOG_ERR("Accel Decoder API not supported");
		return;
	}

	if (sensor_get_decoder(gyro_dev, &decoder) < 0)
	{
		LOG_ERR("Gyro Decoder API not supported");
		return;
	}
	
	LOG_INF("Accelerometer device %p name is %s", accel_dev, accel_dev->name);
	LOG_INF("Gyroscope device %p name is %s", gyro_dev, gyro_dev->name);
	imu->state_info.dev_status = DEV_ONLINE;
	imu->state_info.offline_cnt = 0;
	imu->state_info.offline_max_cnt = 10;
}
/*
*轮询式更新传感器数据（阻塞）
* @param imu 传感器数据结构指针
*/
static void __maybe_unused imu_update(imu_sensor_t *imu)
{
	struct sensor_value acc[3], gyr[3];
	if (imu->state_info.dev_status == DEV_OFFLINE)
	{
		LOG_ERR("IMU device is offline");
		imu->state_info.offline_cnt++;
		return;
	}
	else
	{

		// 读取加速度计数据
		sensor_sample_fetch(accel_dev);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

		// 读取陀螺仪数据
		sensor_sample_fetch(gyro_dev);
		sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
		imu->raw_info.raw_acc_x = sensor_value_to_float(&acc[0]);
		imu->raw_info.raw_acc_y = sensor_value_to_float(&acc[1]);
		imu->raw_info.raw_acc_z = sensor_value_to_float(&acc[2]);
		imu->raw_info.raw_gyr_x = sensor_value_to_float(&gyr[0]);
		imu->raw_info.raw_gyr_y = sensor_value_to_float(&gyr[1]);
		imu->raw_info.raw_gyr_z = sensor_value_to_float(&gyr[2]);

		imu->state_info.offline_cnt = 0;
	}
}
/*
* 接收中断读取的数据，解码传感器数据并打印加速度计数据（非阻塞）
* @param dev 传感器设备指针
* @param buf 传感器数据缓冲区指针
* @param buf_len 传感器数据缓冲区长度
*/
static void decode_data(const struct device *dev, uint8_t *buf, uint32_t buf_len)
{
	const struct sensor_decoder_api *decoder;
	uint8_t accel_buf[128] = {0};
	uint8_t gyro_buf[128] = {0};
	struct sensor_three_axis_data *accel_data = (struct sensor_three_axis_data *)accel_buf;
	struct sensor_three_axis_data *gyro_data = (struct sensor_three_axis_data *)gyro_buf;
	int ret;

	ret = sensor_get_decoder(dev, &decoder);
	if (ret != 0)
	{
		LOG_ERR("Failed to get sensor decoder: %d", ret);
		return;
	}

	if(dev == accel_dev)
	{
		struct sensor_decode_context iter_ctx = SENSOR_DECODE_CONTEXT_INIT(
			decoder, buf, SENSOR_CHAN_ACCEL_XYZ,0);

		while (sensor_decode(&iter_ctx, accel_data, 1) > 0)
		{
			imu_sensor.raw_info.raw_acc_x = sensor_qvalue_to_float(accel_data->readings[0].x,accel_data->shift);
			imu_sensor.raw_info.raw_acc_y = sensor_qvalue_to_float(accel_data->readings[0].y,accel_data->shift);
			imu_sensor.raw_info.raw_acc_z = sensor_qvalue_to_float(accel_data->readings[0].z,accel_data->shift);
			imu_sensor.state_info.offline_cnt = 0;
			
			LOG_INF("Accel data: x=%.6f, y=%.6f, z=%.6f\n", (double)imu_sensor.raw_info.raw_acc_x, (double)imu_sensor.raw_info.raw_acc_y, (double)imu_sensor.raw_info.raw_acc_z);
		}
	}else if (dev == gyro_dev)
	{
		struct sensor_decode_context iter_ctx = SENSOR_DECODE_CONTEXT_INIT(
			decoder, buf, SENSOR_CHAN_GYRO_XYZ,0);

		while (sensor_decode(&iter_ctx, gyro_data, 1) > 0)
		{
			imu_sensor.raw_info.raw_gyr_x = sensor_qvalue_to_float(gyro_data->readings[0].x,gyro_data->shift);
			imu_sensor.raw_info.raw_gyr_y = sensor_qvalue_to_float(gyro_data->readings[0].y,gyro_data->shift);
			imu_sensor.raw_info.raw_gyr_z = sensor_qvalue_to_float(gyro_data->readings[0].z,gyro_data->shift);
			imu_sensor.state_info.offline_cnt = 0;
			
			LOG_INF("Gyro data: x=%.6f, y=%.6f, z=%.6f\n", (double)imu_sensor.raw_info.raw_gyr_x, (double)imu_sensor.raw_info.raw_gyr_y, (double)imu_sensor.raw_info.raw_gyr_z);
		}
	}
}
/*
 * 处理完成队列中的完成事件，获取异步读取的数据并进行解码
 * @param void
 */
static void __maybe_unused process_completions(void)
{
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;

	while (1)
	{
		cqe = rtio_cqe_consume(&ctx);
		if (cqe == NULL)
		{
			k_sleep(K_MSEC(10)); // 主动让出 CPU
			continue;
		}
		if (cqe->result != 0)
		{
			LOG_ERR("async read failed %d\n", cqe->result);
			rtio_cqe_release(&ctx, cqe);
			continue;
		}
		
		const struct device *dev = (const struct device *)cqe->userdata;

		if( rtio_cqe_get_mempool_buffer(&ctx, cqe, &buf, &buf_len) == 0)
		{
			decode_data(dev, buf, buf_len);
			rtio_release_buffer(&ctx, buf, buf_len);
		}
		else
		{
			LOG_ERR("Failed to get buffer from CQE");
		}
		rtio_cqe_release(&ctx, cqe);
		
	}
}

int main()
{
	imu_init(&imu_sensor);

	struct rtio_sqe *handle;
    int ret;
	//启动传感器数据流包括了寄存器配置
	ret = sensor_stream(&accel_iodev, &ctx, (void *)accel_dev, &handle);
	if (ret < 0)
	{
		LOG_ERR("sensor_stream failed: %d", ret);
		return -1;
	}
	ret = sensor_stream(&gyro_iodev, &ctx, (void *)gyro_dev, &handle);
	if (ret < 0)
	{
		LOG_ERR("sensor_stream failed: %d", ret);
		return -1;
	}

	while (1)
	{
		// imu_update(&imu_sensor);
		process_completions(); 
		
	}
}
