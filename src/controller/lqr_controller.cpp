#include "controller/lqr_controller.h"

#include <algorithm>
#include <eigen3/Eigen/LU>

using namespace autopilot::controller;

// Config LQR
void LqrController::LoadControlConf() {
    ts_ = 0.01;    // 每隔0.01s进行一次控制

    cf_ = 155494.663;                              // 前轮侧偏刚度,左右轮之和
    cr_ = 155494.663;                              // 后轮侧偏刚度, 左右轮之和
    wheelbase_ = 2.852;                            
    steer_ratio_ = 16;                             // 方向盘的转角到轮胎转动角度之间的比值系数
    steer_single_direction_max_degree_ = 470.0;    // 最大方向转角

    const double mass_fl = 1845.0/4;                     // 左前悬的质量
    const double mass_fr = 1845.0/4;                     // 右前悬的质量
    const double mass_rl = 1845.0/4;                     // 左后悬的质量
    const double mass_rr = 1845.0/4;                     // 右后悬的质量
    const double mass_front = mass_fl + mass_fr;    // 前悬质量
    const double mass_rear = mass_rl + mass_rr;     // 后悬质量
    mass_ = mass_front + mass_rear;

    lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // 汽车前轮到中心点的距离
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);     // 汽车后轮到中心点的距离

    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;    // 汽车的转动惯量

    lqr_eps_ = 0.01;              // LQR 迭代求解精度
    lqr_max_iteration_ = 1500;    // LQR的迭代次数

    return;
}

// initialize controller
void LqrController::Init() {
    // Matrix init operations.
    const int matrix_size = basic_state_size_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    /*
    A matrix (Gear Drive)
    [0.0,                             1.0,                           0.0,                                            0.0;
     0.0,          (-(c_f + c_r) / m) / v,               (c_f + c_r) / m,                (l_r * c_r - l_f * c_f) / m / v;
     0.0,                             0.0,                           0.0,                                            1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // 初始化A矩阵的常数项
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    // 初始化A矩阵的非常数项
    matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // 初始化B矩阵
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    // 状态向量
    matrix_state_ = Matrix::Zero(matrix_size, 1);
    // 反馈矩阵
    matrix_k_ = Matrix::Zero(1, matrix_size);
    // lqr cost function中 输入值u的权重
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_r_(0, 0) = 10;
    // lqr cost function中 状态向量x的权重
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

    // int q_param_size = 4;
    matrix_q_(0, 0) = 2;    // TODO: lateral_error
    matrix_q_(1, 1) = 1;    // TODO: lateral_error_rate
    matrix_q_(2, 2) = 0.1;    // TODO: heading_error
    matrix_q_(3, 3) = 0.1;    // TODO: heading__error_rate

    matrix_q_updated_ = matrix_q_;

    return;
}