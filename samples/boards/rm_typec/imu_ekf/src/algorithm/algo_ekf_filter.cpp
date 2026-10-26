#include "algo_ekf_filter.hpp"
namespace breeze
{
    EAppStatus CAlgo_Ekf::InitAlgo_(SFilterInitParam_Base &param)
    {
        if(param.AlgoID != EAlgoID::ALGO_IMU_EKF) {
            return APP_ERROR; // 确保传入的参数类型正确
        }
        // 调用基类初始化
        EAppStatus status = CAlgo_Kf::InitAlgo_(param);
        if (status != APP_OK) {
            return status;
        }

        // 将基类参数转换为派生类参数
        SAlgoEKfInitParam &ekf_param = static_cast<SAlgoEKfInitParam&>(param);

        
        Chi_Set_ = ekf_param.Chi_Set;
        // 初始化非线性函数和雅可比函数
        f_ = ekf_param.f;
        h_ = ekf_param.h;
        jacF_ = ekf_param.jacF;
        jacH_ = ekf_param.jacH;
        p_func_ = ekf_param.p_func;
        Xhat_func_ = ekf_param.Xhat_func;
        if (Chi_Set_)
        {
            chi_ = ekf_param.chi;
            k_func_ = ekf_param.k_func;
        }

        Chi_Square_Mat = zeros<float>(1, 1); // 初始化卡方检验矩阵
        Chi_Square = 0.0f; // 初始化卡方值
        Ekf_Info.isitialized = true; // 标记EKF已初始化
        return APP_OK;
    }
    /**
    * @brief 更新状态转移雅可比
    * 
    */
    void  CAlgo_Ekf::algo_EKF_jacF_update()
    {
	    jacF_(xhat, u, DT, F);
	    FT = trans(F);
    }

    /**
     * @brief 更新观测雅可比
     *
     */
    void CAlgo_Ekf::algo_EKF_jacH_update()
    {
	    jacH_(xhatMinus, H);
	    HT = trans(H);
    }

    /**
     * @brief EKF先验估计更新
     *
     */
    void CAlgo_Ekf::Algo_EKf_Xhatminus_Update()
    {
        xhatMinus = f_(xhat, u, DT, F, B);
    }

    /**
     * @brief EKF先验估计协方差更新
     *
     */
    void CAlgo_Ekf::Algo_EKf_Pminus_Update()
    {
        p_func_(Pminus);
        Algo_Kf_Pminus_Update();
    }

    /**
     * @brief EKF增益更新，先调用基类的增益更新再进行动态调整（只有使用卡方时才使用这个函数）
     *
     */
    void CAlgo_Ekf::Algo_EKf_K_Update(void)
    {
        Algo_Kf_K_Update();
        k_func_(K);
    }

    /**
     * @brief EKF后验最优估计更新
     *
     */
    void CAlgo_Ekf::Algo_EKf_Xhat_Update()
    {
        Matrixt<float> Correct = K * (z - h_(xhatMinus));
        Xhat_func_(xhat, xhatMinus, Correct, DT);
    }

    bool CAlgo_Ekf::QuaternionEKF_Chi()
    {
        // 卡方检验
        Matrixt<float> Y = z - h_(xhatMinus);
        S = H * Pminus * HT + R; // 残差协方差
        Matrixt<float> S_inv = inv(S);
        Chi_Square_Mat = trans(Y) * S_inv * Y;
        Chi_Square = Chi_Square_Mat[0][0];
        return chi_(K,P,Pminus, H,R,Chi_Square);
    }

        /**
         * @brief EKF观测向量更新
         *
         */

        // 预测步
        void CAlgo_Ekf::Ekf_Predict()
    {

        // 状态预测（非线性）
        Algo_EKf_Xhatminus_Update();

        // 计算 F 雅可比
        algo_EKF_jacF_update();

        // 协方差预测
        Algo_EKf_Pminus_Update();
    }

    //更新步
    void CAlgo_Ekf::Ekf_Update()
    {

        //计算 H 雅可比
        algo_EKF_jacH_update();

        // 有效观测才做测量更新
        if (measurement_valid_num_ > 0)
        {
            
            if (Chi_Set_)
            {
                Chi_Pass = QuaternionEKF_Chi();
                if(Chi_Pass)
               {
                   Algo_EKf_K_Update();
                   Algo_EKf_Xhat_Update();
                   Algo_Kf_P_Update();
               }
                else
                {
                    xhat = xhatMinus;
                    P = Pminus;
                }
                
            }
            else// 没有设置卡方检验标志则直接更新
            {
                Algo_EKf_Xhat_Update();
                Algo_Kf_P_Update();
            }
        }
        else
        {
            xhat = xhatMinus;
            P = Pminus;
        }
    
        Kf_Info.filtered_value = xhat;
        Ekf_Info.filtered_value = xhat;
    }

    /**
     * @brief 主更新处理函数（在中断里要先调用Set_Measured_Vector更新观测向量，然后调用这个函数进行EKF更新）
     *
     */
    EAppStatus CAlgo_Ekf::UpdateHandler_()
    {
        z = measured_vector_;
        Ekf_Predict();
        Ekf_Update();
        return APP_OK;
    }
}
