/**
 * @file conf_algo.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 算法配置
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef CONF_ALGO_HPP
#define CONF_ALGO_HPP

#include <stdint.h>
#include <map>
#include <vector>
#include <math.h>

namespace breeze {

/**
 * @brief 算法ID枚举类型
 *
 */
enum class EAlgoID {
	ALGO_NULL = -1,   ///< 空算法
	ALGO_IMU_AVE = 0, ///< IMU互补滤波
	ALGO_KF = 1,      ///< 基本卡尔曼滤波
	ALGO_IMU_EKF,     ///< IMU扩展卡尔曼滤波
};

/**
 * @brief Application Status枚举类型
 * @note 用作Application中函数的返回值
 */
enum EAppStatus {
	APP_RESET = 0, ///< 重置
	APP_OK = 1,    ///< 正常
	APP_ERROR,     ///< 错误
	APP_BUSY,      ///< 忙
	APP_TIMEOUT,   ///< 超时
	APP_FULL,      ///< 满
	APP_EMPTY,     ///< 空
	APP_INVALID,   ///< 无效
	APP_UNKNOWN    ///< 未知
};

/**
 * @brief 配置并初始化所有算法
 * @return APP_OK - 初始化成功
 * @return APP_ERROR - 初始化失败
 */
EAppStatus InitAllAlgo();

} // namespace breeze

#endif  // CONF_ALGO_HPP
