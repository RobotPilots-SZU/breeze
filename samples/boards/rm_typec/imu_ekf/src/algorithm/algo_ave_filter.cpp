/**
 * @file algo_ave_filter.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 陀螺仪互补 + mahony滤波
 * @version 1.0
 * @date 2026-01-12
 *
 * @copyright Copyright (c) 2026
 *
 */

// #include "Algorithm.hpp"
// #include "mems/mems_bmi088.hpp"
#include <algo_ave_filter.hpp>
#include <math.h>

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)
#define G          9.7803f
#define X          0
#define Y          1
#define Z          2

namespace breeze
{

// CMemsBase *pmems_ave_test = nullptr;

/**
 * @brief 初始化互补滤波算法
 * @retval EAppStatus
 */
EAppStatus CAlgo_IMU_Ave::InitAlgo_(SFilterInitParam_Base &param)
{

	// 检查param是否正确
	if (param.AlgoID == EAlgoID::ALGO_NULL) {
		return APP_ERROR;
	}

	// 类型转换
	auto &imu_ave_param = static_cast<SAlgoImuAveInitParam &>(param);

	// 传感器指针与参数配置
	mems = MemsIDMap.at(imu_ave_param.memsDevID);
	ALPHA = imu_ave_param.ALPHA;
	DT = imu_ave_param.DT;

	// 调试用
	pmems_ave_test = mems;

	// 启动设备
	mems->StartDevice();

	// 注册算法
	RegisterAlgorithm_();

	return APP_OK;
}

/**
 * @brief 更新Mahony滤波数据
 * @retval EAppStatus
 */
EAppStatus CAlgo_IMU_Ave::UpdateHandler_()
{
	// 检查设备指针和状态
	if (mems == nullptr) {
		return APP_ERROR;
	}

	// 读取原始数据
	float_t ax_raw = mems->memsData[CMemsBase::DATA_ACC_X];
	float_t ay_raw = mems->memsData[CMemsBase::DATA_ACC_Y];
	float_t az_raw = mems->memsData[CMemsBase::DATA_ACC_Z];
	float_t gx_raw = mems->memsData[CMemsBase::DATA_GYRO_X];
	float_t gy_raw = mems->memsData[CMemsBase::DATA_GYRO_Y];
	float_t gz_raw = mems->memsData[CMemsBase::DATA_GYRO_Z];

	// 如果未初始化，先用加速度计计算初始状态
	if (!Imu_Ave_Info.is_initialized) {
		// 确保az不为0以避免atan2分母为0的问题，且加速度测量值有效
		if (az_raw != 0.0f || ax_raw != 0.0f || ay_raw != 0.0f) {
			Imu_Ave_Info.imu_ave_pitch =
				rad2deg(atan2f(ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw)));
			Imu_Ave_Info.imu_ave_roll = rad2deg(atan2f(-ay_raw, az_raw));
			Imu_Ave_Info.imu_ave_yaw = 0.0f; ///< yaw初始化为0

			// 根据初始欧拉角计算四元数
			float_t cy = cosf(0.0f);
			float_t sy = sinf(0.0f);
			float_t cp = cosf(Imu_Ave_Info.imu_ave_pitch * 0.5f * PI / 180.0f);
			float_t sp = sinf(Imu_Ave_Info.imu_ave_pitch * 0.5f * PI / 180.0f);
			float_t cr = cosf(Imu_Ave_Info.imu_ave_roll * 0.5f * PI / 180.0f);
			float_t sr = sinf(Imu_Ave_Info.imu_ave_roll * 0.5f * PI / 180.0f);

			q0 = cr * cp * cy + sr * sp * sy;
			q1 = sr * cp * cy - cr * sp * sy;
			q2 = cr * sp * cy + sr * cp * sy;
			q3 = cr * cp * sy - sr * sp * cy;

			// 初始化加速度滤波值
			Imu_Ave_Info.acc_x_filter = ax_raw;
			Imu_Ave_Info.acc_y_filter = ay_raw;
			Imu_Ave_Info.acc_z_filter = az_raw;

			Imu_Ave_Info.is_initialized = true;
		}
		return APP_OK; // 第一次仅计算初始值，下一次再开始滤波
	}

	// --- Mahony AHRS Algorithm ---
	float z_gyro_bias_rad = 0.003f * PI / 180.0f;

	float gx = -gx_raw;
	float gy = -gy_raw;
	float gz = gz_raw - z_gyro_bias_rad;
	float ax = -ax_raw;
	float ay = -ay_raw;
	float az = az_raw;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (twoKi > 0.0f) {
			exInt += twoKi * halfex * DT;
			eyInt += twoKi * halfey * DT;
			ezInt += twoKi * halfez * DT;
			gx += exInt;
			gy += eyInt;
			gz += ezInt;
		} else {
			exInt = 0.0f;
			eyInt = 0.0f;
			ezInt = 0.0f;
		}

		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	gx *= (0.5f * DT);
	gy *= (0.5f * DT);
	gz *= (0.5f * DT);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// --- End of Mahony algorithm ---

	// 计算姿态角 (单位变为度)
	// 从适配后的右手系Mahony四元数提取角度，并通过取反适配回原工程的欧拉角定义
	Imu_Ave_Info.imu_ave_pitch =
		asinf(2.0f * (q1 * q3 - q0 * q2)) * 180.0f / PI; // 提取出来的其实是 -Pitch_mahony
	Imu_Ave_Info.imu_ave_roll =
		-atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
		180.0f / PI; // 提取 -Roll_mahony
	Imu_Ave_Info.imu_ave_yaw =
		atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f /
		PI; // Yaw 保持不变

	/******************************** 获取加速度 ******************************/
	// 加速度计低通滤波
	const float_t acc_filter_kp = 0.1f;
	Imu_Ave_Info.acc_x_filter += (ax_raw - Imu_Ave_Info.acc_x_filter) * acc_filter_kp;
	Imu_Ave_Info.acc_y_filter += (ay_raw - Imu_Ave_Info.acc_y_filter) * acc_filter_kp;
	Imu_Ave_Info.acc_z_filter += (az_raw - Imu_Ave_Info.acc_z_filter) * acc_filter_kp;

	// 根据融合的姿态计算平动加速度
	float_t accel_x_raw = ax_raw - G * sin(Imu_Ave_Info.imu_ave_pitch / 180.f * PI);
	float_t accel_y_raw = ay_raw + G * sin(Imu_Ave_Info.imu_ave_roll / 180.f * PI) *
					       cos(Imu_Ave_Info.imu_ave_pitch / 180.f * PI);
	float_t accel_z_raw = az_raw - G * cos(Imu_Ave_Info.imu_ave_roll / 180.f * PI) *
					       cos(Imu_Ave_Info.imu_ave_pitch / 180.f * PI);

	// 一阶低通滤波
	float_t kp = 0.04f;
	const float ay_offset = -0.2f;

	Imu_Ave_Info.accel_x += (accel_x_raw - Imu_Ave_Info.accel_x) * kp;
	Imu_Ave_Info.accel_y += (accel_y_raw - ay_offset - Imu_Ave_Info.accel_y) * kp;
	Imu_Ave_Info.accel_z += (accel_z_raw - Imu_Ave_Info.accel_z) * kp;

	return APP_OK;
}

} // namespace breezze
