/**
 * @file algo_filter_common.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 定义滤波器基类
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ALGO_FILTER_COMMON_HPP
#define ALGO_FILTER_COMMON_HPP

#include "conf_algo.hpp"

namespace breeze{
    
/**
 * @brief 滤波器基类
 * 
 */
class CFilterBase{
// 声明友元函数
friend void StartUpdateTask(void *argument);
protected:

    /**
     * @brief 定义一个专门用于被继承的基类结构体
     * 
     */
    struct SFilterInitParam_Base
    {
	    EAlgoID AlgoID = EAlgoID::ALGO_NULL; ///< 算法ID
    };

    // 初始化
    virtual EAppStatus InitAlgo_(SFilterInitParam_Base &param) {return APP_ERROR;}

    // 更新处理
    virtual EAppStatus UpdateHandler_() {return APP_ERROR;}

    // 创建算法任务
    virtual EAppStatus CreateAlgorithmTask_() { return APP_ERROR; }

    // 注册算法
    EAppStatus RegisterAlgorithm_();

    // 注销算法
    EAppStatus UnregisterAlgorithm_();

public:
    EAlgoID AlgoID;     ///< 算法ID

    // 算法构造函数
    CFilterBase() = default;

    // 算法析构函数
    virtual ~CFilterBase() {UnregisterAlgorithm_();};
};

extern std::map<EAlgoID, CFilterBase*> AlgoIDMap;

} // namespace breeze

#endif // ALGO_FILTER_COMMON_HPP
