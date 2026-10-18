/**
 * @file rp_matrix.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 矩阵库
 * @version 1.0
 * @date 2026-01-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef RP_MATRIX_HPP
#define RP_MATRIX_HPP

#include <cmath>
/*需开启的K_config*/
// # 启用CMSIS-DSP库
// CONFIG_CMSIS_DSP=y
// CONFIG_CMSIS_DSP_MATRIX=y
// CONFIG_STD_CPP17=y
#include "arm_math.h"
#include <memory>
#include <type_traits>

    using ARM_MAT_INS = arm_matrix_instance_f32;

namespace breeze{

template<typename T>
class Matrixt; // 前向声明矩阵类

template<typename T>
Matrixt<T> zeros(int rows, int cols);

/**
 * @brief 矩阵类模板
 * @param 元素类型T
 *
 */
template <typename T>
class Matrixt{
public:
    // 默认构造
    Matrixt() : rows_(0), cols_(0), data_(nullptr){
        if constexpr (std::is_same_v<T,float>){
            arm_mat_init_f32(&arm_mat_, 0, 0, nullptr);
        }
    }

    // 根据传入的行 列维度构造
    Matrixt(int rows, int cols) : rows_(rows), cols_(cols), data_(std::make_unique<T[]>(rows * cols)){  ///< 初始化列表
        if constexpr (std::is_same_v<T, float>){
            arm_mat_init_f32(&arm_mat_, rows_, cols_, (float32_t *)data_.get());    ///< 初始化矩阵实例
        }
    }

    // 移动构造(右值引用)
    Matrixt(Matrixt&& mat) noexcept
        : rows_(mat.rows_), cols_(mat.cols_), data_(std::move(mat.data_)) {     // 转移unique_ptr所有权
        if constexpr (std::is_same_v<T, float>) {
            arm_mat_init_f32(&arm_mat_, rows_, cols_, (float32_t*)data_.get());
        }
        mat.rows_ = 0;
        mat.cols_ = 0;
    }

    // 移动赋值(右值引用)
    Matrixt<T>& operator=(Matrixt<T>&& mat) noexcept {
        if (this != &mat) {
            rows_ = mat.rows_;
            cols_ = mat.cols_;
            data_ = std::move(mat.data_);       // 转移unique_ptr所有权
            if constexpr (std::is_same_v<T, float>) {
                arm_mat_init_f32(&arm_mat_, rows_, cols_, (float32_t*)data_.get());
            }
            mat.rows_ = 0;
            mat.cols_ = 0;
        }
        return *this;
    }

    // 拷贝构造函数
    Matrixt(const Matrixt<T>& mat) 
        : rows_(mat.rows_), cols_(mat.cols_),
          data_(std::make_unique<T[]>(mat.rows_ * mat.cols_)) {
        // 深拷贝数据
        std::copy(mat.data_.get(), mat.data_.get() + mat.size(), data_.get());
        
        if constexpr (std::is_same_v<T, float>) {
            arm_mat_init_f32(&arm_mat_, rows_, cols_, (float32_t*)data_.get());
        }
    }

    // 析构函数为默认 当data_超出作用域时会自动调用delete
    ~Matrixt() = default;

    int rows(void) const {return this->rows_;}    ///< 返回行维度
    int cols(void) const {return this->cols_;}    ///< 返回列维度

    /***************** 符号重载start ********************/
    // 取矩阵元素
    T* operator[](const int& row) { return &this->data_[row * cols_]; }
    const T* operator[](const int& row) const { return &this->data_[row * cols_]; }

    /**
     * @brief 赋值运算符重载(拷贝赋值)
     * @param 同维度矩阵mat的引用
     * 
     */
    Matrixt<T>& operator=(const Matrixt<T>& mat) {
        if(this == &mat){
            return *this;   ///< 如果发现是自己给自己赋值那就直接返回
        }
        if(this->rows_ != mat.rows_ || this->cols_ != mat.cols_){    ///< 如果维度不匹配就重新分配内存
            this->rows_ = mat.rows_;
            this->cols_ = mat.cols_;
            this->data_ = std::make_unique<T[]>(rows_ * cols_);
        }
        // 复制数据
        std::copy(mat.data_.get(),mat.data_.get() + (mat.rows_ * mat.cols_), this->data_.get());
        if constexpr (std::is_same_v<T, float>) {
            arm_mat_init_f32(&this->arm_mat_, this->rows_, this->cols_, (float32_t*)this->data_.get());
        }   // 更新arm_mat_
        return *this;
    }

    /**
     * @brief 加法赋值运算符重载
     * @param 同维度的矩阵mat
     * 
     */
    Matrixt<T>& operator+=(const Matrixt<T>& mat) {
        if(rows_ != mat.rows_ || cols_ != mat.cols_){
            return *this;   // 维度不匹配则返回自身
        }   // 确保维度匹配
        arm_status s;
        (void)s;
        if constexpr (std::is_same_v<T, float>){    // 浮点矩阵用dsp库加速
            s = arm_mat_add_f32(&this->arm_mat_, mat.get_arm_mat(), &this->arm_mat_);
        }
        else{
            for(int i = 0;i < rows_ * cols_; i++){
                this->get_data()[i] += mat.get_data()[i];
            }
        }
        return *this;
    }

    /**
     * @brief 减法运算符重载
     * @param 同维度的矩阵mat
     * 
     */
    Matrixt<T>& operator-=(const Matrixt<T>& mat) {
        if(rows_ != mat.rows_ || cols_ != mat.cols_){
            return *this;   // 维度不匹配则返回自身
        }
        if constexpr (std::is_same_v<T, float>){    // 浮点矩阵用dsp库加速
            arm_status s = arm_mat_sub_f32(&this->arm_mat_, mat.get_arm_mat(), &this->arm_mat_);
            (void)s;
        }
        else{
            for(int i = 0;i < rows_ * cols_; i++){
                this->get_data()[i] -= mat.get_data()[i];
            }
        }
        return *this;
    }

    /**
     * @brief 矩阵标量乘法赋值运算符重载
     * @tparam 标量类型U
     * @param 乘数val
     */
    template <typename U>
    Matrixt<T>& operator*=(const U& val) {
        if constexpr (std::is_same_v<T, float> && std::is_same_v<U, float>) {
            arm_status s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
            (void)s;
        } else {
            for (int i = 0; i < rows_ * cols_; i++) {
                data_[i] *= val;
            }
        }
        return *this;
    }

    /**
     * @brief 标量除法赋值运算符重载
     * @tparam 标量类型U
     * @param 除数val
     */
    template <typename U>
    Matrixt<T>& operator/=(const U& val) {
        const U eps = std::is_floating_point_v<U> ? U(1e-6) : U(0);
        if (std::abs(val) < eps) {
            return *this;   // 除零则返回自身
        }     ///< 确保分母大于零
        if constexpr (std::is_same_v<T, float> && std::is_same_v<U, float>) {
            arm_status s = arm_mat_scale_f32(&this->arm_mat_, 1.0f / val, &this->arm_mat_);
            (void)s;
        } else {
            for (int i = 0; i < rows_ * cols_; i++) {
                data_[i] /= val;
            }
        }
        return *this;
    }

    /**
     * @brief 矩阵乘法运算符重载
     * @param 乘数矩阵mat
     * 
     */
    Matrixt<T> operator*(const Matrixt<T>& mat) const {
        if(this->cols_ != mat.rows_){
            return zeros<T>(0, 0);   // 维度不匹配则返回零矩阵
        } // 矩阵乘法维度检查
        Matrixt<T> res(this->rows_, mat.cols_);

        if constexpr (std::is_same_v<T, float>) {
            arm_status s = arm_mat_mult_f32(&this->arm_mat_, mat.get_arm_mat(), res.get_arm_mat());
            (void)s;
        }
        else {
            // 通用矩阵乘法 (i, j, k)
            for (int i = 0; i < res.rows_; i++) {
                for (int j = 0; j < res.cols_; j++) {
                    T sum = 0;
                    for (int k = 0; k < this->cols_; k++) {
                        sum += (*this)[i][k] * mat[k][j];
                    }
                    res[i][j] = sum;
                }
            }
        }
        return res;
    }

    /**
    * @brief 矩阵除法运算符重载
    * @param 除数矩阵mat 满足方阵且可逆
    *
    */
    Matrixt<T> operator/(const Matrixt<T>& mat) const {
        if (mat.rows_ != mat.cols_ || this->cols_ != mat.rows_) {
            return zeros<T>(0, 0); // 维度不匹配返回空矩阵
        }

        Matrixt<T> mat_inv = inv(mat);
        // 若除数矩阵不可逆则返回恐惧真
        if (mat_inv.rows_ == 0 || mat_inv.cols_ == 0) {
            return zeros<T>(0, 0);
        }

        Matrixt<T> res(this->rows_, mat.cols_);
        if constexpr (std::is_same_v<T, float>) {
            arm_status s = arm_mat_mult_f32(&this->arm_mat_, mat_inv.get_arm_mat(), res.get_arm_mat());
            (void)s;
        }
        else {
            // 矩阵乘法实现相除
            for (int i = 0; i < res.rows_; i++) {
                for (int j = 0; j < res.cols_; j++) {
                    T sum = 0;
                    for (int k = 0; k < this->cols_; k++) {
                        sum += (*this)[i][k] * mat_inv[k][j];
                    }
                    res[i][j] = sum;
                }
            }
        }
        return res;
    }

    /**
     * @brief 加法运算符重载
     * @param 同维度的矩阵mat
     * 
     */
    Matrixt<T> operator+(const Matrixt<T>& mat) const{
        if(rows_ != mat.rows_ || cols_ != mat.cols_){
            return zeros<T>(0, 0);   // 维度不匹配则返回零矩阵
        }    ///< 维度检查
        Matrixt<T> res(rows_, cols_);

        if constexpr (std::is_same_v<T, float>) {           ///< 如果是浮点矩阵就调用dsp库的加速算法
            arm_status s = arm_mat_add_f32(&this->arm_mat_, mat.get_arm_mat(), res.get_arm_mat());
            (void)s;
        }
        else{
            for(int i = 0; i < rows_ * cols_; i++){
                res.get_data()[i] = this->get_data()[i] + mat.get_data()[i];    ///< 用for循环加法
            }
        }
        
        return res;
    }

    /**
     * @brief 减法运算符重载
     * @param 同维度的矩阵mat
     * 
     */
    Matrixt<T> operator-(const Matrixt<T>& mat) const{
        if(rows_ != mat.rows_ || cols_ != mat.cols_){
            return zeros<T>(0, 0);   // 维度不匹配则返回零矩阵
        }    ///< 维度检查
        Matrixt<T> res(rows_, cols_);

        if constexpr (std::is_same_v<T, float>){            ///< 如果是浮点矩阵就调用dsp库的加速算法
            arm_status s = arm_mat_sub_f32(&this->arm_mat_, mat.get_arm_mat(), res.get_arm_mat());
            (void)s;
        }
        else{
            for(int i = 0; i < rows_ * cols_; i++){
                res.get_data()[i] = this->get_data()[i] - mat.get_data()[i];    ///< 用for循环加法
            }
        }

        return res;
    }

    /***************** 符号重载end ********************/

    /* @brief 取子矩阵
    * 
    * @param start_row   从原矩阵哪行开始截取
    * @param start_col  从原矩阵哪列开始截取
    * @param block_row   子矩阵行数
    * @param block_col   子矩阵列数
    * 
    */
    Matrixt<T> block(int start_row, int start_col, int block_row, int block_col) const {
        
        if(start_row < 0 || start_col < 0 || start_row + block_row > this->rows_ || start_col + block_col > this->cols_){
            return Matrixt<T>(0, 0);  // 边界检查，确保要提取的块在原矩阵内部，越界则返回0×0矩阵
        }
        Matrixt<T> res(block_row, block_col);
        for (int i = 0; i < block_row; ++i) {
            for (int j = 0; j < block_col; ++j) {
                res[i][j] = (*this)[start_row + i][start_col + j];
            }
        }

        return res;
    }

    // 创建行向量
    Matrixt<T> row(int row) const{
        return this->block(row, 0, 1, this->cols_);
    }

    // 创建列向量
    Matrixt<T> col(int col) const{
        return this->block(0, col, this->rows_, 1);
    }

    // 判断是否为空矩阵
    bool empty() const { return rows_ == 0 || cols_ == 0; }

    // 获取元素总数
    size_t size() const { return rows_ * cols_; }

    inline T* get_data() { return data_.get();}                ///< 获取可读可写的原始指针
    inline const T* get_data() const { return data_.get();}    ///< 获取只可读的原始指针

    inline ARM_MAT_INS* get_arm_mat() { return &arm_mat_; }    ///< 获取可读可写的原始指针
    inline const ARM_MAT_INS* get_arm_mat() const { return &arm_mat_; }    ///< 获取只可读的原始指针

private:
    int rows_;                  ///< 行维度
    int cols_;                  ///< 列维度
    std::unique_ptr<T[]> data_; ///< 矩阵指针
    ARM_MAT_INS arm_mat_;       ///< 使用官方库的矩阵实例 用于硬件加速

};  // class Matrixt end

// 矩阵相关函数

// 零矩阵
template<typename T>
Matrixt<T> zeros(int rows, int cols){
    Matrixt<T> res(rows, cols);
    std::fill(res.get_data(), res.get_data() + rows * cols, 0);
    return res;
}

// 全1矩阵
template<typename T>
Matrixt<T> ones(int rows, int cols){
    Matrixt<T> res(rows, cols);
    std::fill(res.get_data(), res.get_data() + rows * cols, 1);
    return res;
}

/**
 * @brief 单位矩阵
 * @param 方阵维度
 * 
 */
template<typename T>
Matrixt<T> eye(int dims){
    Matrixt<T> res(dims, dims);
    std::fill(res.get_data(), res.get_data() + dims * dims, 0);
    for(int i = 0; i < dims; i++){
        res.get_data()[i * dims + i] = T(1);  // 把对角线元素改成1
    }
    return res;
}

// 矩阵转置
template<typename T>
Matrixt<T> trans(const Matrixt<T>& mat) {
    Matrixt<T> res(mat.cols(), mat.rows());

    if constexpr (std::is_same_v<T, float>) {
        arm_mat_trans_f32(mat.get_arm_mat(), res.get_arm_mat());
    } 
    else {
        // 非浮点类型手动转置
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
                res[j][i] = mat[i][j];
            }
        }
    }
    return res;
}

// 求矩阵的迹
template<typename T>
float trace(const Matrixt<T>& mat) { 
    if(mat.rows() != mat.cols()){
        return 0; // 确保是方阵
    }
    float res = 0;
    for(int i = 0; i < mat.rows(); i++){
        res += mat[i][i];
    }
    return res;
}

// 求矩阵范数
template<typename T>
float norm(const Matrixt<T>& mat) {
    if(!(std::is_floating_point_v<T>) || !(mat.rows() == 1 || mat.cols() == 1)){
        return 0;     // 确保为浮点类型且是向量（1行或1列）
    }
    
    T sum = T(0);
    int len = mat.rows() * mat.cols();
    const T* data = mat.get_data();
    // 计算平方和
    for (int i = 0; i < len; ++i) {
        sum += data[i] * data[i];
    }
    // 返回平方根
    return std::sqrt(sum);
}

/**
 * @brief 矩阵求逆 (高斯-若尔当消元法)
 * @tparam 元素类型T(仅支持float或double类型)
 * @param 需要求逆的方阵mat
 * @retval 返回mat的逆矩阵 如果矩阵奇异则返回同维度的零矩阵
 */
template <typename T>
Matrixt<T> inv(const Matrixt<T>& mat) {
    const int dim = mat.rows();
    if(dim != mat.cols()){
        return zeros<T>(0, 0); // 求逆操作只对方阵有效 否则返回零矩阵
    }
    arm_status s;   // 运算状态
    (void)s;

    // 仅支持浮点类型求逆，非浮点直接返回零矩阵
    if constexpr (!std::is_floating_point_v<T>) {
        return zeros<T>(0, 0);
    }

    Matrixt<T> res(dim, dim);

    if constexpr(std::is_same_v<T, float>){
        s = arm_mat_inverse_f32(mat.get_arm_mat(), res.get_arm_mat());
        if (s == ARM_MATH_SINGULAR) {   // 矩阵奇异
            return zeros<T>(dim, dim);  // 返回同维度零矩阵
        }
    }
    else{   // double类型
        // 构造增广矩阵 [A|I]
        Matrixt<T> ext_mat = zeros<T>(dim, 2 * dim);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                ext_mat[i][j] = mat[i][j]; // 复制 A 到左半部分
            }
            ext_mat[i][dim + i] = 1; // 在右半部分放置单位阵 I
        }

        // 高斯-若尔当消元
        for (int i = 0; i < dim; i++) {
            // 部分主元法：找到当前列绝对值最大的元素作为主元
            T abs_max = std::fabs(ext_mat[i][i]);
            int abs_max_row = i;
            for (int row = i + 1; row < dim; row++) {
                if (std::fabs(ext_mat[row][i]) > abs_max) {
                    abs_max = std::fabs(ext_mat[row][i]);
                    abs_max_row = row;
                }
            }

            // 若T是double类型则阈值为1e-9，若为整形则阈值为0
            const T eps = std::is_floating_point_v<T> ? T(1e-9) : T(0);     ///< 判断阈值
            // 如果主元过小，则认为矩阵奇异，不可逆
            if (abs_max < eps) {
                s = ARM_MATH_SINGULAR;
                return zeros<T>(dim, dim); // 返回零矩阵表示失败
            }

            // 将主元所在行与当前行交换
            if (abs_max_row != i) {
                for (int col = 0; col < 2 * dim; col++) {
                    std::swap(ext_mat[i][col], ext_mat[abs_max_row][col]);
                }
            }

            // 将主元归一化为1
            T pivot = ext_mat[i][i];
            for (int col = i; col < 2 * dim; col++) {
                ext_mat[i][col] /= pivot;
            }

            // 将其他行的当前列消为0
            for (int row = 0; row < dim; row++) {
                if (row != i) {
                    T factor = ext_mat[row][i];
                    for (int col = i; col < 2 * dim; col++) {
                        ext_mat[row][col] -= factor * ext_mat[i][col];
                    }
                }
            }
        }

        // 从增广矩阵的右半部分提取结果 即mat的逆
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                res[i][j] = ext_mat[i][dim + j];
            }
        }
    }
    
    s = ARM_MATH_SUCCESS;
    return res;
}

/**
 * @brief 帽子矩阵 即反对称矩阵
 * @tparam 变量类型T
 * @param 矩阵mat(注意一定得是3×1列向量)
 * @retval mat的反对称矩阵
 */
template<typename T>
Matrixt<T> hat(const Matrixt<T>& mat) {
  if(mat.rows() != 3 || mat.cols() != 1){
    return zeros<T>(0, 0);   // 维度不匹配则返回零矩阵
  }
  Matrixt<T> res(3, 3);
    const T* data = mat.get_data();
    const T v0 = data[0];
    const T v1 = data[1];
    const T v2 = data[2];

    res[0][0] = 0;  res[0][1] = -v2; res[0][2] = v1;
    res[1][0] = v2; res[1][1] = 0;   res[1][2] = -v0;
    res[2][0] = -v1;res[2][1] = v0;  res[2][2] = 0;

    return res;
}

/**
 * @brief hat的逆运算，将一个3x3反对称矩阵转换为一个3x1向量
 * @tparam 元素类型T
 * @param 3x3的反对称矩阵mat
 * @return 对应的3x1向量
 */
template<typename T>
Matrixt<T> vee(const Matrixt<T>& mat) {
    if(mat.rows() != 3 || mat.cols() != 3){
        return zeros<T>(0, 0); // 必须是3x3矩阵
    }

    Matrixt<T> res(3, 1);
    
    res[0][0] = mat[2][1];
    res[1][0] = mat[0][2];
    res[2][0] = mat[1][0];

    return res;
}

/**
 * @brief 叉乘
 * @tparam 变量类型T
 * @param 矩阵mat1，mat2
 * @retval mat1和mat2叉乘结果
 */
template<typename T>
Matrixt<T> cross(const Matrixt<T>& mat1, const Matrixt<T>& mat2) {
  return hat(mat1) * mat2;
}


} // namespace breeze

#endif // RP_MATRIX_HPP
