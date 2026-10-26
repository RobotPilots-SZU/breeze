/**
 * @file Algorithm.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 算法层的汇总源文件
 * @version 1.0
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <algo_kf_filter.hpp>

namespace breeze {

std::map<EAlgoID, CFilterBase*> AlgoIDMap;  ///< 算法map容器

/**
 * @brief 注册算法
 * 
 * @return EAppStatus 
 */
EAppStatus CFilterBase::RegisterAlgorithm_() {
    
    if (AlgoID == EAlgoID::ALGO_NULL) return APP_ERROR;

    AlgoIDMap.insert(std::make_pair(AlgoID, this));
    return APP_OK;
}

/**
 * @brief 注销算法
 * 
 * @return EAppStatus 
 */
EAppStatus CFilterBase::UnregisterAlgorithm_() {
    
    if (AlgoID == EAlgoID::ALGO_NULL) return APP_ERROR;

    AlgoIDMap.erase(AlgoID);
    return APP_OK;
}

} // namespace breeze
