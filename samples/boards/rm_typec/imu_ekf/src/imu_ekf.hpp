/********************************************************************************
 * @brief        IMU_EKF姿态解算
 * @file         imu_ekf.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-5-30
 * @copyright    Copyright (c) 2025
 ********************************************************************************/

#ifndef IMU_EKF_HPP
#define IMU_EKF_HPP
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "algo_ekf_filter.hpp"

namespace breeze
{

/* 陀螺仪坐标变换为云台坐标结构体 */
typedef struct {
	float arz;
	float ary;
	float arx;
	float trans[9];
} gimbal_transform_t;

//云台坐标下的姿态角
typedef struct
{
	float G_Roll;
	float G_Pitch;
	float G_Yaw;
	float YawTotalAngle = 0.0f;
	int16_t YawRoundCount = 0;
} gimbal_info_t;

typedef struct
{
	float raw_acc_x;
	float raw_acc_y;
	float raw_acc_z;
	float raw_gyr_x;
	float raw_gyr_y;
	float raw_gyr_z;

} raw_info_t;

//初始化时陀螺仪零偏估计结构体
typedef struct
{
	float gx_offset;
	float gy_offset;
	float gz_offset;
} offset_info_t;

typedef enum
{
	IMU_NONE_ERR,
	IMU_TYPE_ERR,
	IMU_ID_ERR,
	IMU_INIT_ERR,
	IMU_DATA_ERR,
	IMU_DATA_CALI,
} imu_err_e;

typedef struct work_state_struct
{
	imu_err_e err_code;

} work_state_t;


Matrixt<float> EKF_F_XhatMinus_Update(Matrixt<float> &xhat, Matrixt<float> &u, float dt,
									  Matrixt<float> &F, Matrixt<float> &B);
Matrixt<float> EKF_H_Update(const Matrixt<float> &xhatMinus);
void EKF_jacF_Update(Matrixt<float> &xhat, Matrixt<float> &u, float dt, Matrixt<float> &F);
void EKF_jacH_Update(const Matrixt<float> &xhatMinus, Matrixt<float> &H);
void EKF_P_Update(Matrixt<float> &P);
bool EKF_Chi_Update(Matrixt<float> &K, Matrixt<float> &P, const Matrixt<float> &Pminus, 
					const Matrixt<float> &H, const Matrixt<float> &R, float Chi);
void EKF_K_Update(Matrixt<float> &K);
void EKF_Xhat_Update(Matrixt<float> &xhat, const Matrixt<float> &xhatMinus, Matrixt<float> &correct, float dt);

class Imu_Sensor_t 
{
public:
	float Q1;      // 四元数更新过程噪声
	float Q2;      // 陀螺仪零偏过程噪声
	float R; // 加速度计量测噪声
	float ChiSquareTestThreshold; // 卡方检验阈值
	float lambda;               // 渐消因子
	
	float Gyro[3];
	float Accel[3];
	float GyroBias[3]; // 陀螺仪零偏估计值

	float ChiSquare_Data;          ///< 当前观测残差的卡方统计量
	uint16_t ErrorCount; // 卡方错误计数
	float OrientationCosine[3];            // 各个轴的方向余弦用于动态更新k
	float AdaptiveGainScale;            //根据卡方动态调整增益的缩放因子
	bool StableFlag;            // 姿态稳定标志
	bool ChiConverge;            // 卡方检验是否收敛

	raw_info_t raw_info;
	offset_info_t offset_info;
	gimbal_info_t gimbal_info;
	work_state_t work_state;
	

	CAlgo_Ekf imu_ekf; // EKF实例

	Imu_Sensor_t();
	~Imu_Sensor_t() noexcept = default;

	EAppStatus Ekf_Info_Init()
	{
		// 初始化EKF参数
		CAlgo_Ekf::SAlgoEKfInitParam ekf_param;
		ekf_param.AlgoID = EAlgoID::ALGO_IMU_EKF;
		ekf_param.DT = dt;
		ekf_param.x_size = 6; // 四元数(4) + 陀螺仪零偏(2)
		ekf_param.u_size = 0; // 
		ekf_param.z_size = 3; // 加速度计测量(3) 
		ekf_param.f = EKF_F_XhatMinus_Update;
		ekf_param.h = EKF_H_Update;
		ekf_param.jacF = EKF_jacF_Update;
		ekf_param.jacH = EKF_jacH_Update;
		ekf_param.p_func = EKF_P_Update;
		ekf_param.Xhat_func = EKF_Xhat_Update;
		ekf_param.Chi_Set = true; // 设置卡方检验标志
		ekf_param.chi = EKF_Chi_Update; // 设置卡方检验函数
		ekf_param.k_func = EKF_K_Update; // 设置增益更新函数

		// Base KF 初始化时仍检查这些向量长度
		ekf_param.use_auto_adjustment = false;
		EAppStatus status = imu_ekf.InitAlgo_(ekf_param);
		work_state.err_code = (status == APP_OK) ? IMU_DATA_CALI : IMU_INIT_ERR;
		return status; // 如需，可在后续添加错误处理
	}
	
	void Dt_Update(float new_dt) {
		this->dt = new_dt;
		imu_ekf.Set_DT(new_dt);
	}
	
	void Raw_Info_Update(float gx, float gy, float gz, float ax, float ay, float az);
	void Ekf_Info_Update();
	void Info_Print();

private:
	float q[4];		 // 四元数估计值
	
	float dt; // 姿态更新周期

	float gyro_norm;
	float accl_norm;

	float YawAngleLast;
};

extern Imu_Sensor_t imu_sensor;
void Imu_Init(Imu_Sensor_t &imu);
void Imu_Update(Imu_Sensor_t &imu);

} // namespace breeze

#endif // IMU_EKF_H
