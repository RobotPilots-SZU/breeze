#include <cmath>
#include <arm_math.h>
#include "imu_ekf.hpp"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

namespace breeze{
Imu_Sensor_t imu_sensor;

//坐标系变化矩阵
ARM_MAT_INS EKFTrans;
ARM_MAT_INS EKFSrc;
ARM_MAT_INS EKFDst;

gimbal_transform_t EKFgim_trans = {
	.arz = 0.0f,
	.ary = 0.0f,
	.arx = 0.0f,
	.trans = {0.0f},
};

/*
 * @brief Imu_Sensor_t 构造函数，初始化成员变量
 *
- @param[in] Q1 设置四元数的过程噪声协方差矩阵，越小解算数据越平滑，越大系统对快速变化的反应越快   10
- @param[in] Q2 设置陀螺仪零偏估计过程噪声协方差矩阵     0.001
- @param[in] R  设置加速度计测量噪声协方差矩阵，越小对加速度越信任，系统对快速变化的反应越快       1000000
- @param[in] lambda         设置渐消因子防止陀螺仪零偏估计协方差过度收敛         0.9996
*/
Imu_Sensor_t::Imu_Sensor_t()
	: Q1(10.0f), Q2(0.001f), R(1000000.0f),
	  ChiSquareTestThreshold(3.5e-8f), lambda(0.9996f),
	  Gyro{0.0f, 0.0f, 0.0f}, Accel{0.0f, 0.0f, 0.0f}, GyroBias{0.0f, 0.0f, 0.0f},
	  ChiSquare_Data(0.0f), ErrorCount(0), OrientationCosine{0.0f, 0.0f, 0.0f}, AdaptiveGainScale(0.0f),
	  StableFlag(false), ChiConverge(false),
	  raw_info{},offset_info{},gimbal_info{}, work_state{},
	 q{0.0f, 0.0f, 0.0f, 0.0f}, dt(0.01f), gyro_norm(0.0f), accl_norm(0.0f), YawAngleLast(0.0f)
{}

float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
								 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
								 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
								 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
								 0.1, 0.1, 0.1, 0.1, 100, 0.1,
								 0.1, 0.1, 0.1, 0.1, 0.1, 100};

/*--------self define end-------------------------------------------*/


/**
 * @brief 初始云台坐标系
 */
void transform_init(gimbal_transform_t *gim_trans)
{
	float arz, ary, arx;

	/* 角度单位转换（to弧度） */
	arz = gim_trans->arz * 0.017453f;
	ary = gim_trans->ary * 0.017453f;
	arx = gim_trans->arx * 0.017453f;

	/* 旋转矩阵赋值（三个旋转矩阵叠加） */
gim_trans->trans[0] = cosf(arz) * cosf(ary);
	gim_trans->trans[1] = cosf(arz) * sinf(ary) * sinf(arx) -
		      sinf(arz) * cosf(arx);
	gim_trans->trans[2] = cosf(arz) * sinf(ary) * cosf(arx) +
		      sinf(arz) * sinf(arx);
	gim_trans->trans[3] = sinf(arz) * cosf(ary);
	gim_trans->trans[4] = sinf(arz) * sinf(ary) * sinf(arx) +
		      cosf(arz) * cosf(arx);
	gim_trans->trans[5] = sinf(arz) * sinf(ary) * cosf(arx) -
		      cosf(arz) * sinf(arx);
	gim_trans->trans[6] = -sinf(ary);
	gim_trans->trans[7] = cosf(ary) * sinf(arx);
	gim_trans->trans[8] = cosf(ary) * cosf(arx);

	/* 3x3变换矩阵初始化 */
	arm_mat_init_f32(&EKFTrans, 3, 3, (float *)gim_trans->trans);
}

/**
 * @brief  将陀螺仪坐标变换为云台坐标，若不需要变换可在imu_protocol.c中imu_update将其注释
 * @brief  坐标变换采用Z-Y-X欧拉角描述，即从陀螺仪坐标系向云台坐标系变换中，坐标系按照绕陀螺仪Z轴、Y轴、X轴的顺序旋转
 *					每一次旋转的参考坐标系为当前陀螺仪坐标系
 * @param[in]  (float) gx,  gy,  gz,  ax,  ay,  az
 * @param[out] (float *) gx, gy, gz, aax, ay, az
 */
void Vector_Transform(float gx, float gy, float gz,
					  float ax, float ay, float az,
					  float *ggx, float *ggy, float *ggz,
					  float *aax, float *aay, float *aaz)
{
	/* 陀螺仪输入输出数组定义 */
	float gyro_in[3], gyro_out[3];
	/* 加速度输入输出数组定义 */
	float acc_in[3], acc_out[3];

	/* 陀螺仪赋值 */
	gyro_in[0] = (float)gx, gyro_in[1] = (float)gy, gyro_in[2] = (float)gz;
	/* 加速度计赋值 */
	acc_in[0] = (float)ax, acc_in[1] = (float)ay, acc_in[2] = (float)az;

	/* 陀螺仪坐标变换 */
	arm_mat_init_f32(&EKFSrc, 1, 3, gyro_in);
	arm_mat_init_f32(&EKFDst, 1, 3, gyro_out);
	arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
	*ggx = gyro_out[0], *ggy = gyro_out[1], *ggz = gyro_out[2];

	/* 加速度计坐标变换 */
	arm_mat_init_f32(&EKFSrc, 1, 3, acc_in);
	arm_mat_init_f32(&EKFDst, 1, 3, acc_out);
	arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
	*aax = acc_out[0], *aay = acc_out[1], *aaz = acc_out[2];
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,单位化四元数
 * @param
 *
 */
void EKF_jacF_Update(Matrixt<float> &xhat, Matrixt<float> &u, float dt, Matrixt<float> &F)
{
	// 单位化
	float q0, q1, q2, q3;
	q0 = xhat[0][0];
	q1 = xhat[1][0];
	q2 = xhat[2][0];
	q3 = xhat[3][0];
	float norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;
	xhat[0][0] = q0;
	xhat[1][0] = q1;
	xhat[2][0] = q2;
	xhat[3][0] = q3;

	F[0][4] = q1 * dt / 2.0f;
	F[0][5] = q2 * dt / 2.0f;
	F[1][4] = -q0 * dt / 2.0f;
	F[1][5] = q3 * dt / 2.0f;
	F[2][4] = -q3 * dt / 2.0f;
	F[2][5] = -q0 * dt / 2.0f;
	F[3][4] = q2 * dt / 2.0f;
	F[3][5] = -q1 * dt / 2.0f;
}


void EKF_jacH_Update(const Matrixt<float> &xhatMinus, Matrixt<float> &H)
{
	H = zeros<float>(3, 6); 
	float doubleq0, doubleq1, doubleq2, doubleq3;
	doubleq0 = 2.0f * xhatMinus[0][0];
	doubleq1 = 2.0f * xhatMinus[1][0];
	doubleq2 = 2.0f * xhatMinus[2][0];
	doubleq3 = 2.0f * xhatMinus[3][0];

	H[0][0] = -doubleq2;
	H[0][1] = doubleq3;
	H[0][2] = -doubleq0;
	H[0][3] = doubleq1;
	H[1][0] = doubleq1;
	H[1][1] = doubleq0;
	H[1][2] = doubleq3;
	H[1][3] = doubleq2;	
	H[2][0] = doubleq0;
	H[2][1] = -doubleq1;
	H[2][2] = -doubleq2;
	H[2][3] = -doubleq3;
}

void EKF_P_Update(Matrixt<float> &P)
{
	float lambda = imu_sensor.lambda;

	P[4][4] /= lambda; 
	P[5][5] /= lambda; 
	if (P[4][4] > 10000)
	{
		P[4][4] = 10000;
	}
	if (P[5][5] > 10000)
	{
		P[5][5] = 10000;
	}
}

// 先设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了非线性计算先验xhat
Matrixt<float> EKF_F_XhatMinus_Update(Matrixt<float> &xhat, Matrixt<float> &u, float dt, Matrixt<float> &F, Matrixt<float> &B)
{
	Matrixt<float> xhatMinus(6, 1);
	float halfgxdt, halfgydt, halfgzdt;
	float Gyro[3] = {imu_sensor.Gyro[0], imu_sensor.Gyro[1], imu_sensor.Gyro[2]};
	// set F
	/*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
	F = eye<float>(6); 
	halfgxdt = 0.5f * Gyro[0] * dt;
	halfgydt = 0.5f * Gyro[1] * dt;
	halfgzdt = 0.5f * Gyro[2] * dt;

	F[0][1] = -halfgxdt;
	F[0][2] = -halfgydt;
	F[0][3] = -halfgzdt;
	F[1][0] = halfgxdt;
	F[1][2] = halfgzdt;
	F[1][3] = -halfgydt;
	F[2][0] = halfgydt;
	F[2][1] = -halfgzdt;
	F[2][3] = halfgxdt;
	F[3][0] = halfgzdt;
	F[3][1] = halfgydt;
	F[3][2] = -halfgxdt;	

	xhatMinus = F * xhat;
	float quat_norm = sqrt(xhatMinus[0][0] * xhatMinus[0][0] + xhatMinus[1][0] * xhatMinus[1][0] +
			xhatMinus[2][0] * xhatMinus[2][0] + xhatMinus[3][0] * xhatMinus[3][0]);
	if (quat_norm > 1e-6f && !std::isnan(quat_norm)) {
		xhatMinus[0][0] /= quat_norm;
		xhatMinus[1][0] /= quat_norm;
		xhatMinus[2][0] /= quat_norm;
		xhatMinus[3][0] /= quat_norm;
	} else {
		xhatMinus[0][0] = 1.0f;
		xhatMinus[1][0] = 0.0f;
		xhatMinus[2][0] = 0.0f;
		xhatMinus[3][0] = 0.0f;
	}
	return xhatMinus;
};

Matrixt<float> EKF_H_Update(const Matrixt<float> &xhatMinus)
{
	Matrixt<float> h(3, 1);
	float q0, q1, q2, q3;
	q0 = xhatMinus[0][0];
	q1 = xhatMinus[1][0];	
	q2 = xhatMinus[2][0];
	q3 = xhatMinus[3][0];

	h[0][0] = 2.0f * (q1 * q3 - q0 * q2);
	h[1][0] = 2.0f * (q0 * q1 + q2 * q3);
	h[2][0] = q0 * q0 - q1 * q1 -q2 * q2 + q3 * q3;
	for (uint8_t i = 0; i < 3; ++i)
	{
		imu_sensor.OrientationCosine[i] = std::acos(std::abs(h[i][0]));
	}
	return h;	
}

/*
 * @brief EKF卡方检验更新
 * @param	Matrixt<float> &K 增益矩阵
 * @param	Matrixt<float> &P 协方差矩阵
 * @param	Matrixt<float> &Pminus 先验协方差矩阵
 * @param	Matrixt<float> &H 观测矩阵
 * @param	Matrixt<float> &R 测量噪声协方差矩阵
 * @param	float Chi 卡方统计量
 * @return	bool 检验结果
 */
bool EKF_Chi_Update(Matrixt<float> &K, Matrixt<float> &P,const Matrixt<float> &Pminus,const Matrixt<float> &H, const Matrixt<float> &R, float Chi)
{
	float threshold = imu_sensor.ChiSquareTestThreshold;
	bool stable = imu_sensor.StableFlag;
	bool &converge = imu_sensor.ChiConverge;
	static uint16_t error_count = 0;
	uint8_t x_size_ = P.rows();
	//后面更新k使用
	imu_sensor.ChiSquare_Data=Chi;

	// 情况1：
	if (Chi < threshold * 0.5f)
	{
		converge = true;
		error_count = 0;
	}

	// 情况2：超出阈值且此前收敛
	if (Chi > threshold && converge)
	{	
		if (stable)
		{
			error_count++;
			if (error_count > 50)
			{
				// 判定发散
				converge = false;
				error_count = 0;

				Matrixt<float> I = eye<float>(x_size_);
				Matrixt<float> A = I - K * H;
				P = A * Pminus * trans(A) + K * R * trans(K);
				return true;
			}
		}
		else
		{
			error_count = 0;
		}

		// 未达到发散条件，不更新（只用预测值）
		return false;
	}

	error_count = 0;
	return true;
}

/*
 * @brief EKF动态增益更新，要先调用Algo_Kf_K_Update()再动态调整
 * @param	Matrixt<float> &K 增益矩阵
 */
void EKF_K_Update(Matrixt<float> &K)
{
	bool converge = imu_sensor.ChiConverge;
	float Chi = imu_sensor.ChiSquare_Data;
	float threshold = imu_sensor.ChiSquareTestThreshold;
	float &adaptive_gain_scale = imu_sensor.AdaptiveGainScale;

	// 计算自适应增益
	if (Chi > 0.1f * threshold && converge)
	{
		adaptive_gain_scale = (threshold - Chi) / (0.9f * threshold);
	}
	else
	{
		adaptive_gain_scale = 1.0f;
	}
	

	// 应用自适应增益缩放
	for (int i = 0; i < K.rows(); ++i)
	{
		for (int j = 0; j < K.cols(); ++j)
		{
			K[i][j] *= adaptive_gain_scale;
		}
	}

	// 姿态角相关增益的特殊处理
	for (int i = 4; i < 6; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			K[i][j] *= imu_sensor.OrientationCosine[i - 4] / 1.5707963f;
		}
	}
}

/*
 * @brief EKF状态更新，包含单位化四元数和零漂修正限幅
 * @param	Matrixt<float> &xhat 当前状态估计值
 * @param	Matrixt<float> &xhatMinus 先验状态估计值
 * @param	Matrixt<float> &correct 卡尔曼增益乘以测量残差的校正项
 * @param	float dt 更新周期
 */
void EKF_Xhat_Update(Matrixt<float> & xhat, const Matrixt<float> &xhatMinus, Matrixt<float> &correct, float dt)
{
	//零漂修正限幅,一般不会有过大的漂移
	for (uint8_t i = 4; i < 6; ++i)
	{
		if (correct[i][0] > 1e-2f*dt)
		{
			correct[i][0] = 1e-2f*dt;
		}
		else if (correct[i][0] < -1e-2f*dt)
		{
			correct[i][0] = -1e-2f*dt;
		}
		
	}
	// 不修正q3轴数据防止耦合
	//correct[3][0] = 0.0f;
	xhat = xhatMinus + correct;
	
	// 单位化四元数部分
	float quat_norm = sqrt(xhat[0][0] * xhat[0][0] + xhat[1][0] * xhat[1][0] +
						   xhat[2][0] * xhat[2][0] + xhat[3][0] * xhat[3][0]);
	if (quat_norm > 1e-6f && !std::isnan(quat_norm))
	{
		xhat[0][0] /= quat_norm;
		xhat[1][0] /= quat_norm;
		xhat[2][0] /= quat_norm;
		xhat[3][0] /= quat_norm;
	}
	else
	{
		xhat[0][0] = 1.0f;
		xhat[1][0] = 0.0f;
		xhat[2][0] = 0.0f;
		xhat[3][0] = 0.0f;
	}
}
/**
 * @brief  中断中调用原始数据更新函数
 * @param	float gx, float gy, float gz, float ax, float ay, float az
 *
 */
void Imu_Sensor_t::Raw_Info_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	Gyro[0] = gx-GyroBias[0];
	Gyro[1] = gy-GyroBias[1];
	Gyro[2] = gz-GyroBias[2];
	gyro_norm = sqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
	float accelInvNorm;
	Accel[0] = ax;
	Accel[1] = ay;
	Accel[2] = az;
	accl_norm = sqrt(ax * ax + ay * ay + az * az);
	Matrixt<float> accel_measure(3, 1);
	if (accl_norm > 1e-6f && !std::isnan(accl_norm)) {
		accelInvNorm = 1.0f / accl_norm;
		accel_measure[0][0] = ax * accelInvNorm;
		accel_measure[1][0] = ay * accelInvNorm;
		accel_measure[2][0] = az * accelInvNorm;
		imu_ekf.Set_Measured_Vector(accel_measure);
		// 只有在加速度有效时才认为姿态稳定 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
		if (accl_norm > 9.8f - 5.5f && accl_norm < 9.8f + 5.5f && gyro_norm < 2.0f) {
			StableFlag = true;
		} else {
			StableFlag = false;
		}
	} else {
		StableFlag = false;
	}
	
}

static int64_t imu_tick_last = 0; 
static float imu_dt;
/**
 * @brief  更新EKF融合结果，包含Q,R更新debug可直接改结构体变量调参
 * @param
 *
 */
void Imu_Sensor_t::Ekf_Info_Update()
{
	//setQ,R
	Matrixt<float> Q = zeros<float>(6, 6);
	Q[0][0] = Q1*dt;
	Q[1][1] = Q1*dt;
	Q[2][2] = Q1*dt;
	Q[3][3] = Q1*dt;
	Q[4][4] = Q2*dt;
	Q[5][5] = Q2*dt;
	imu_ekf.Set_Q(Q);

	Matrixt<float> R_mat = zeros<float>(3, 3);
	R_mat[0][0] = R;
	R_mat[1][1] = R;
	R_mat[2][2] = R;
	imu_ekf.Set_R(R_mat);

	//set DT
	if (imu_tick_last == 0)
	{
		imu_tick_last = k_uptime_get() - 1; // 首次调用，设置初始时间为1ms前
	}
	imu_dt = k_uptime_delta(&imu_tick_last) * 0.001f; // 毫秒转秒
	this->Dt_Update(imu_dt);

	imu_ekf.UpdateHandler_();
	// 获取融合后的数据,包括四元数和xy零飘值
	q[0] = imu_ekf.Ekf_Info.filtered_value[0][0];
	q[1] = imu_ekf.Ekf_Info.filtered_value[1][0];
	q[2] = imu_ekf.Ekf_Info.filtered_value[2][0];
	q[3] = imu_ekf.Ekf_Info.filtered_value[3][0];
	GyroBias[0] = imu_ekf.Ekf_Info.filtered_value[4][0];
	GyroBias[1] = imu_ekf.Ekf_Info.filtered_value[5][0];
	GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

	// 利用四元数反解欧拉角
	float Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
	float Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
	float sintemp, costemp;
	sintemp = -2.0f * (q[1] * q[3] - q[0] * q[2]);
	costemp = sqrt(1 - sintemp * sintemp);
	float Pitch = atan2f(sintemp, costemp);
	Yaw *=  57.295779513f;
	Roll *= 57.295779513f;
	Pitch *= 57.295779513f;

	gimbal_info.G_Roll = Roll;
	gimbal_info.G_Pitch = Pitch;
	gimbal_info.G_Yaw = Yaw;
	// get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
	if (Yaw - YawAngleLast > 180.0f)
	{
		gimbal_info.YawRoundCount--;
	}
	else if (Yaw - YawAngleLast < -180.0f)
	{
		gimbal_info.YawRoundCount++;
	}

	gimbal_info.YawTotalAngle = 360.0f * gimbal_info.YawRoundCount + Yaw;
	YawAngleLast = Yaw;
}

void Imu_Sensor_t::Info_Print()
{
	LOG_INF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", (double)gimbal_info.G_Roll, (double)gimbal_info.G_Pitch, (double)gimbal_info.G_Yaw);
}
} // namespace breeze
//----------------------------------以下为包含具体设备读取的驱动---------------------------------------
extern struct device *accel_dev;
extern struct device *gyro_dev;

namespace breeze{

/** @brief  初始化EKF状态估计值
 *  @param
 *
 */
static void imu_ekf_xhat_init()
{
	struct sensor_value acc[3], gyr[3];
	float acc_sum[3] = {0};
	float acc_init[3];
	for (uint8_t i = 0; i < 100; ++i)
	{
		// 读取加速度计数据
		sensor_sample_fetch(accel_dev);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

		// 读取陀螺仪数据
		sensor_sample_fetch(gyro_dev);
		sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
		acc_sum[0] += sensor_value_to_float(&acc[0]);
		acc_sum[1] += sensor_value_to_float(&acc[1]);
		acc_sum[2] += sensor_value_to_float(&acc[2]);
		k_sleep(K_MSEC(1));
	}
	for (uint8_t i = 0; i < 3; ++i)
	{
		acc_init[i] = acc_sum[i] / 100.0f;
	}

	float pitch = atan2(-acc_init[0], sqrt(acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2]));
	float roll = atan2(acc_init[1], acc_init[2]);

	float half_pitch = pitch / 2.0f;
	float half_roll = roll / 2.0f;

	Matrixt<float> xhat(6, 1);
	// 初始化四元数为单位四元数 [1, 0, 0, 0]
	xhat[0][0] = cos(half_pitch) * cos(half_roll); // q0
	xhat[1][0] = cos(half_pitch) * sin(half_roll); // q1
	xhat[2][0] = sin(half_pitch) * cos(half_roll); // q2
	xhat[3][0] = sin(half_pitch) * sin(half_roll); // q3
	xhat[4][0] = 0.0f;							   // bias_x
	xhat[5][0] = 0.0f;							   // bias_y
	imu_sensor.imu_ekf.Set_xhat(xhat);			   // 设置初始状态估计值为零
}

/** @brief  初始化EKF协方差矩阵
 *  @param
 *
 */
static void imu_ekf_P_init()
{
	Matrixt<float> P(6, 6);
	for (uint8_t i = 0; i < 36; ++i)
	{
		P[i / 6][i % 6] = IMU_QuaternionEKF_P[i];
	}
	imu_sensor.imu_ekf.Set_P(P);
}

/** @brief  初始化IMU传感器
 *  @param
 *
 */
void Imu_Init(Imu_Sensor_t &imu)
{
	EAppStatus status = imu.Ekf_Info_Init();
	if (status != APP_OK)
	{
		LOG_ERR("Failed to initialize EKF: %d", static_cast<int>(status));
		return;
	}

	if (!device_is_ready(accel_dev))
	{
		LOG_ERR("Accelerometer device %s is not ready", accel_dev->name);
		return;
	}

	if (!device_is_ready(gyro_dev))
	{
		LOG_ERR("Gyroscope device %s is not ready", gyro_dev->name);
		return;
	}

	LOG_INF("Gyroscope device %p name is %s", gyro_dev, gyro_dev->name);

	imu_ekf_xhat_init();
	imu_ekf_P_init();
	transform_init(&EKFgim_trans);
}

/*
 *@brief 读取IMU原始数据并更新EKF融合结果（注意初始化时IMU_DATA_CALI状态时不能移动，不让会导致陀螺仪偏移量计算错误）
 */
void Imu_Update(Imu_Sensor_t &imu)
{
	static float gyrox, gyroy, gyroz;
	static float accx, accy, accz;
	static float gyro[3], accel[3];
	struct sensor_value acc[3], gyr[3];
    static int16_t cali_count = 0;
	// 读取加速度计数据
	sensor_sample_fetch(accel_dev);
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

	// 读取陀螺仪数据
	sensor_sample_fetch(gyro_dev);
	sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	accel[0] = sensor_value_to_float(&acc[0]);
	accel[1] = sensor_value_to_float(&acc[1]);
	accel[2] = sensor_value_to_float(&acc[2]);
	gyro[0] = sensor_value_to_float(&gyr[0]);
	gyro[1] = sensor_value_to_float(&gyr[1]);
	gyro[2] = sensor_value_to_float(&gyr[2]);

	imu.raw_info.raw_acc_x = accel[0];
	imu.raw_info.raw_acc_y = accel[1];
	imu.raw_info.raw_acc_z = accel[2];
	imu.raw_info.raw_gyr_x = gyro[0];
	imu.raw_info.raw_gyr_y = gyro[1];
	imu.raw_info.raw_gyr_z = gyro[2];

	/* 坐标系变换 */
	Vector_Transform(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],
					 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);

	if(imu.work_state.err_code == IMU_DATA_CALI)
	{
		if(cali_count <2000)
		{
			imu.offset_info.gx_offset -= gyrox * 0.0005f;
			imu.offset_info.gy_offset -= gyroy * 0.0005f;
			imu.offset_info.gz_offset -= gyroz * 0.0005f;
			cali_count++;
		}else
		{
			cali_count = 0;
			imu.work_state.err_code = IMU_NONE_ERR;
			
			if(fabsf(imu.offset_info.gx_offset) > 0.005f)
				imu.offset_info.gx_offset = 0;
			if (abs(imu.offset_info.gy_offset) > 0.005f)
				imu.offset_info.gy_offset = 0;
			if (abs(imu.offset_info.gz_offset) > 0.005f)
				imu.offset_info.gz_offset = 0;
		}
	}else
	{
		gyroz += imu.offset_info.gz_offset;
	} 


	imu.Raw_Info_Update(gyrox, gyroy, gyroz,accx, accy, accz);
	imu.Ekf_Info_Update();
	imu.Info_Print();
}

}// namespace breeze

