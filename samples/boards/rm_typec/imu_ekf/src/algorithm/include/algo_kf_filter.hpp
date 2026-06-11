/**
 * @file algo_kf_filter.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 卡尔曼滤波
 * @version 1.0
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ALGO_KF_FILTER_HPP
#define ALGO_KF_FILTER_HPP

#include <algo_filter_common.hpp>
#include <rp_matrix.hpp>

namespace breeze{

/**
 * @brief 卡尔曼滤波算法类
 * 
 */
class CAlgo_Kf: public CFilterBase{
public:
    // 继承基类初始化结构体
    struct SAlgoKfInitParam : public SFilterInitParam_Base{
        
        float_t DT = 0.0f;                          ///< 调度周期
        uint8_t z_size = 0;                         ///< 观测量维度
        uint8_t u_size = 0;                         ///< 输入值维度
        uint8_t x_size = 0;                         ///< 状态量维度

        // 新增：用于动态调整的参数
        bool use_auto_adjustment = false;               ///< 是否开启H, R, K矩阵的动态调整
        std::vector<uint8_t> measurement_map;           ///< 观测量到状态量的映射
        std::vector<float> measurement_degree;          ///< 观测量与状态量的缩放关系(用于构建H)
        std::vector<float> r_diagonal_elements;         ///< R矩阵的对角线元素(用于构建R)
        std::vector<float> state_min_variance;          ///< P矩阵对角线元素的最小值，防止过度收敛
    };
    
    // 卡尔曼滤波信息结构体+实例
    struct SAlgoKfInfo{
        bool isitialized = false;                   ///< 是否完成初始化
        Matrixt<float> filtered_value;              ///< 滤波输出
    } Kf_Info;

    
    CAlgo_Kf() = default;  ///< 默认构造函数

    explicit CAlgo_Kf(SFilterInitParam_Base &param){
        InitAlgo_(param);
    }   ///< 带参的构造函数，用初始化结构体构造

    // 模块析构函数
	~CAlgo_Kf() noexcept { UnregisterAlgorithm_(); };

    /**
     * @brief 设置状态转移矩阵
     * @param 状态转移矩阵F
     * 
     */
    void Set_F(const Matrixt<float>& F) { this->F = F; this->FT = trans(this->F);}

    /**
     * @brief 设置控制矩阵
     * @param 控制矩阵B
     * 
     */
    void Set_B(const Matrixt<float>& B) { this->B = B; }

    /**
     * @brief 设置过程噪声协方差矩阵
     * @param 过程噪声协方差矩阵Q
     * 
     */
    void Set_Q(const Matrixt<float>& Q) { this->Q = Q; }

    void Set_H(const Matrixt<float>& H_in);
    void Set_R(const Matrixt<float>& R_in);

    /**
     * @brief 设置原始测量向量 (在读取传感器数据处调用)
     * @param 所有传感器的最新读数 measured_vector
     */
    void Set_Measured_Vector(const Matrixt<float>& measured_vector) {
        if (measured_vector.rows() == z_size_ && measured_vector.cols() == 1) {
            this->measured_vector_ = measured_vector;
        }
    }

    /**
     * @brief 设置初始状态
     * @param 状态估计向量xhat_in
     */
    void Set_xhat(const Matrixt<float>& xhat_in) { this->xhat = xhat_in; }

    /**
     * @brief 设置调度周期
     * @param 调度周期dt
     */
    void Set_DT(float dt) { this->DT = dt; }

    /**
     * @brief 设置后验估计协方差矩阵，使系统刚开始更快收敛
     * @param 后验估计协方差矩阵P
     */
    void Set_P(const Matrixt<float>& P_in) { this->P = P_in; }
    
    virtual EAppStatus InitAlgo_(SFilterInitParam_Base &param) ;   ///< 初始化

    virtual EAppStatus UpdateHandler_() ;    ///< 更新

protected:
    float_t DT = 0.0f; ///< 调度周期
    // Matrixs
    Matrixt<float> xhat;            ///< 后验最优估计
    Matrixt<float> xhatMinus;       ///< 先验估计
    Matrixt<float> Pminus;          ///< 先验估计协方差矩阵
    Matrixt<float> P;               ///< 后验估计协方差矩阵
    Matrixt<float> F, FT;           ///< 状态转移矩阵及其转置
    Matrixt<float> B;               ///< 控制矩阵
    Matrixt<float> Q;               ///< 过程噪声协方差矩阵
    Matrixt<float> H, HT;           ///< 观测矩阵及其转置
    Matrixt<float> R;               ///< 测量噪声协方差矩阵
    Matrixt<float> K;               ///< 卡尔曼增益矩阵
    Matrixt<float> S;               ///< 观测残差协方差矩阵
    Matrixt<float> z;               ///< 观测向量
    Matrixt<float> u;               ///< 输入向量

    // Size
    uint8_t u_size_;                ///< 输入量维度
    uint8_t x_size_;                ///< 状态量维度
    uint8_t z_size_;                ///< 观测量维度

    // Auto Adjustment related
    bool use_auto_adjustment_;                      ///< 是否启用动态调整
    uint8_t measurement_valid_num_;                 ///< 当前周期有效测量数
    Matrixt<float> measured_vector_;                ///< 原始测量向量(由传感器填充)
    std::vector<uint8_t> measurement_map_;          ///< 观测量->状态量缩放倍数
    std::vector<float> measurement_degree_;         ///< 观测矩阵H构建系数
    std::vector<float> r_diagonal_elements_;        ///< 过程误差R构建系数
    std::vector<float> state_min_variance_;         ///< P的最小方差

    void Algo_Kf_Xhatminus_Update(void);
    void Algo_Kf_Pminus_Update(void);
    void Algo_Kf_K_Update(void);
    void Algo_Kf_Xhat_Update(void);
    void Algo_Kf_P_Update(void);
    void Algo_Kf_Adjustment(void);

};
    
} // namespace breeze

#endif // ALGO_KF_FILTER_HPP
