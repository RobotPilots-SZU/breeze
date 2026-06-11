/**
 * @file algo_kf_filter.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 卡尔曼滤波
 * @version 1.0
 * @date 2026-01-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <algo_kf_filter.hpp>

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace breeze{
    
/**
 * @brief 初始化卡尔曼滤波
 * @retval EAppStatus
 * 
 */
EAppStatus CAlgo_Kf::InitAlgo_(SFilterInitParam_Base &param){

    // 检查param是否正确
    if (param.AlgoID != EAlgoID::ALGO_KF && param.AlgoID != EAlgoID::ALGO_IMU_EKF) {
        return APP_ERROR;
    }

    // 类型转换
    auto &kfparam = static_cast<SAlgoKfInitParam&>(param);

    DT = kfparam.DT;
    u_size_ = kfparam.u_size;
    x_size_ = kfparam.x_size;
    z_size_ = kfparam.z_size;
    use_auto_adjustment_ = kfparam.use_auto_adjustment;
    if(use_auto_adjustment_){
    measurement_degree_ = kfparam.measurement_degree;
    measurement_map_ = kfparam.measurement_map;
    r_diagonal_elements_ = kfparam.r_diagonal_elements;
    state_min_variance_ = kfparam.state_min_variance;
    }else{
        measurement_valid_num_ = z_size_;
    }
    
    // 初始化卡尔曼黄金五式参与运算的矩阵
    xhat = zeros<float>(x_size_, 1);
    xhatMinus = zeros<float>(x_size_, 1);
    F = zeros<float>(x_size_, x_size_);
    FT = zeros<float>(x_size_, x_size_);
    P = eye<float>(x_size_);
    Pminus = zeros<float>(x_size_, x_size_);
    Q = zeros<float>(x_size_, x_size_);
    if (u_size_ > 0) {      // 系统有输入的时候才初始化控制矩阵和输入向量
        B = zeros<float>(x_size_, u_size_);
        u = zeros<float>(u_size_, 1);
    }
    H = zeros<float>(z_size_, x_size_);
    HT = zeros<float>(x_size_, z_size_);
    R = zeros<float>(z_size_, z_size_);
    K = zeros<float>(x_size_, z_size_);
    S = zeros<float>(z_size_, z_size_);
    z = zeros<float>(z_size_, 1);
    measured_vector_ = zeros<float>(z_size_,1);
    if (use_auto_adjustment_)
    {
    if (measurement_map_.size() != z_size_)return APP_ERROR;
    if (measurement_degree_.size() != z_size_)return APP_ERROR;
    if (r_diagonal_elements_.size() != z_size_) return APP_ERROR;
    if (!state_min_variance_.empty() &&state_min_variance_.size() != x_size_)return APP_ERROR;
    }
        // 检查维度
    Kf_Info.isitialized = true;
    return APP_OK;
    }

/**
 * @brief 卡尔曼滤波更新
 * @retval EAppStatus
 */
EAppStatus CAlgo_Kf::UpdateHandler_() {
    // 动态调整(若需要)
    Algo_Kf_Adjustment();

    // 预测
    Algo_Kf_Xhatminus_Update();
    Algo_Kf_Pminus_Update();

    // 有效观测才做测量更新
    if (measurement_valid_num_ > 0) {
        Algo_Kf_K_Update();
        Algo_Kf_Xhat_Update();
        Algo_Kf_P_Update();
    } else {
        xhat = xhatMinus;
        P = Pminus;
    }

    // 4. P限幅
    if (!state_min_variance_.empty()) {
        for (uint8_t i = 0; i < x_size_; ++i) {
            if (P[i][i] < state_min_variance_[i]) {
                P[i][i] = state_min_variance_[i];
            }
        }
    }

    Kf_Info.filtered_value = xhat;
    return APP_OK;
}

/**
 * @brief 更新先验估计
 * 
 */
void CAlgo_Kf::Algo_Kf_Xhatminus_Update(){
    if (u_size_ > 0) {
    xhatMinus = F * xhat + B * u;
    } 
    else {
        xhatMinus = F * xhat;
    }   // 是否有输入量决定X先验协方差的计算方式
}

/**
 * @brief 更新先验估计协方差
 * 
 */
void CAlgo_Kf::Algo_Kf_Pminus_Update(){
    Pminus = F * P * FT + Q;
}

/**
 * @brief 更新卡尔曼增益
 * 
 */
void CAlgo_Kf::Algo_Kf_K_Update(void){
    S = H * Pminus * HT + R;
    Matrixt<float> S_inv = inv(S);
    K = Pminus * HT * S_inv;
}

/**
 * @brief 更新后验最优估计
 * 
 */
void CAlgo_Kf::Algo_Kf_Xhat_Update(void){
    xhat = xhatMinus + K * (z - H * xhatMinus);
}

/**
 * @brief 更新后验估计协方差
 * 
 */
void CAlgo_Kf::Algo_Kf_P_Update(void){
    Matrixt<float>I;
    I = eye<float>(x_size_);
    Matrixt<float> A = I - K * H;
    P = A * Pminus * trans(A) + K * R * trans(K);
}

/**
 * @brief 动态调整
 * 
 * 
 */
void CAlgo_Kf::Algo_Kf_Adjustment() {
    if (!use_auto_adjustment_) {
        // 不用动态调整，直接用原始尺寸
        z = measured_vector_;
        measurement_valid_num_ = z_size_;
        // H、R由外部设置
        return;
    }

    measurement_valid_num_ = 0;
    std::vector<uint8_t> valid_indices;
    for (uint8_t i = 0; i < z_size_; ++i) {
        if (!std::isnan(measured_vector_[i][0])) {
            valid_indices.push_back(i);
            measurement_valid_num_++;
        }
    }
    if (measurement_valid_num_ == 0) return;

    // 动态重构 z, H, R
    z = Matrixt<float>(measurement_valid_num_, 1);
    H = Matrixt<float>(measurement_valid_num_, x_size_);
    R = Matrixt<float>(measurement_valid_num_, measurement_valid_num_);

    std::fill(H.get_data(), H.get_data() + H.size(), 0.0f);
    std::fill(R.get_data(), R.get_data() + R.size(), 0.0f);

    for (uint8_t i = 0; i < measurement_valid_num_; ++i) {
        uint8_t idx = valid_indices[i];
        z[i][0] = measured_vector_[idx][0];
        if (measurement_map_[idx] == 0) continue;
        uint8_t state_idx = measurement_map_[idx] - 1;
        if (state_idx < x_size_) {
            H[i][state_idx] = measurement_degree_[idx];
        }
        R[i][i] = r_diagonal_elements_[idx];
    }
    // 清空 measured_vector_
    std::fill(measured_vector_.get_data(), measured_vector_.get_data() + measured_vector_.size(), NAN);
    HT = trans(H);
}

/**
 * @brief 更新观测矩阵
 * 
 */
void CAlgo_Kf::Set_H(const Matrixt<float>& H_in){
    if (H_in.rows() == z_size_ && H_in.cols() == x_size_) {
        H = H_in;
        HT = trans(H);
    }
}

/**
 * @brief 更新观测噪声协方差
 * 
 */
void CAlgo_Kf::Set_R(const Matrixt<float>& R_in){
    if (R_in.rows() == z_size_ && R_in.cols() == z_size_) {
        R = R_in;
    }
}
    
} // namespace breeze
