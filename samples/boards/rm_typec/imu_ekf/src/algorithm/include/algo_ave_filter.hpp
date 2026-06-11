/**
 * @file algo_ave_filter.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 互补滤波
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ALGO_AVE_FILTER_HPP
#define ALGO_AVE_FILTER_HPP

#include <algo_filter_common.hpp>

namespace breeze
{

/**
 * @brief 互补滤波算法类
 * 
 */
class CAlgo_IMU_Ave final: public CFilterBase{
public:
    // 继承基类初始化结构体
    struct SAlgoImuAveInitParam : public SFilterInitParam_Base
    {
        float_t ALPHA = 0.0f;                         ///< 陀螺仪信任系数
        float_t DT = 0.0f;                            ///< 调度周期
    };

    // 定义互补滤波信息结构体+实例
    struct SAlgoImuAveInfo
    {
        bool is_initialized = false;                ///< 是否完成初始化
        float_t imu_ave_roll = 0.0f;                  ///< roll轴
        float_t imu_ave_pitch = 0.0f;                 ///< pitch轴
        float_t imu_ave_yaw = 0.0f;                   ///< yaw轴
        float_t acc_x_filter = 0.f;                 ///< x轴加速度滤波值
        float_t acc_y_filter = 0.f;                 ///< y轴加速度滤波值
        float_t acc_z_filter = 0.f;                 ///< z轴加速度滤波值
        float_t accel_y = 0.f;                      ///< y轴平动加速度
        float_t accel_x = 0.f;                      ///< x轴平动加速度
        float_t accel_z = 0.f;                      ///< z轴平动加速度
    } Imu_Ave_Info;


    float_t ALPHA = 0.0f;                           ///< 陀螺仪信任系数
    float_t DT = 0.0f;                              ///< 调度周期

    // Mahony滤波参数及变量
    float_t twoKp = 2.5f;                           ///< 2 * proportional gain (Kp)
    float_t twoKi = 0.0015f;                           ///< 2 * integral gain (Ki)
    float_t q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; ///< 四元数
    float_t exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   ///< 积分误差
    
    CAlgo_IMU_Ave() = default;  ///< 默认构造函数

    explicit CAlgo_IMU_Ave(SFilterInitParam_Base &param) {InitAlgo_(param);};   ///< 带参的构造函数，用初始化结构体构造

    // 模块析构函数
	~CAlgo_IMU_Ave() final { UnregisterAlgorithm_(); };

    EAppStatus InitAlgo_(SFilterInitParam_Base &param) final;   ///< 初始化

    /**
     * @brief 设置原始测量向量 (在读取传感器数据处调用)
     * @param 所有传感器的最新读数 measured_vector
     */
    void Set_Measured_Vector(const Matrixt<float> &measured_vector)
    {
	    if (measured_vector.rows() == z_size_ && measured_vector.cols() == 1) {
		    this->measured_vector_ = measured_vector;
	    }
    }

    EAppStatus UpdateHandler_() final;    ///< 更新

};

} // namespace breeze

#endif // ALGO_AVE_FILTER_HPP
