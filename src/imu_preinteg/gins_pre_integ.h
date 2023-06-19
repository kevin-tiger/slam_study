#pragma once
#include "common/basetype.h"
#include "imu_preinteg/imu_preintegration.h"
#include "sensor/gnss.h"
#include "sensor/imu.h"
#include "sensor/odom.h"
#include "common/nav_state.h"
#include "common/math_utils.h"
#include "g2o_opti/g2o_types.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>

/**
 * 使用预积分优化的GINS
 *
 * 本章仍使用两帧模型，不会去做全量优化（尽管也可以做），两帧模型和ESKF更像一些
 * IMU到达时，累计到预积分器中
 * 每次RTK到达时，触发一次优化，并做边缘化。边缘化结果写入下一帧先验中。
 * 里程计方面，使用最近时间的里程计信息作为速度观测量。
 */

class GinsPreInteg {
   public:
    /// GINS 配置项
    struct Options {
        Options() {}

        Vector3d gravity_ = Vector3d(0, 0, -9.8);  // 重力方向

        /// IMU相关噪声参数在preinteg内部配置
        IMUPreintegration::Options preinteg_options_;

        // 噪声
        double bias_gyro_var_ = 1e-6;           // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;           // 加计零偏游走标准差
        Matrix3d bg_rw_info_ = Matrix3d::Identity();  // 陀螺随机游走信息阵
        Matrix3d ba_rw_info_ = Matrix3d::Identity();  // 加计随机游走信息阵

        double gnss_pos_noise_ = 0.1;                   // GNSS位置方差
        double gnss_height_noise_ = 0.1;                // GNSS高度方差
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS角度方差
        Matrix<double, 6, 6> gnss_info_ = Matrix<double, 6, 6>::Identity();           // 6D gnss信息矩阵

        /// 轮速计相关
        double odom_var_ = 0.05;
        Matrix3d odom_info_ = Matrix3d::Identity();
        double odom_span_ = 0.1;        // 里程计测量间隔
        double wheel_radius_ = 0.155;   // 轮子半径
        double circle_pulse_ = 1024.0;  // 编码器每圈脉冲数

        bool verbose_ = true;  // 是否输出调试信息
    };

    /// Option 可以在构造时设置，也可以在后续设置
    GinsPreInteg(Options options = Options()) : options_(options) { SetOptions(options_); }

    /**
     * IMU 处理函数，要求初始零偏已经设置过，再调用此函数
     * @param imu imu读数
     */
    void AddImu(const IMU& imu);

    /**
     * GNSS 处理函数
     * @param gnss
     */
    void AddGnss(const GNSS& gnss);

    /**
     * 轮速计处理函数
     * @param odom
     */
    void AddOdom(const Odom& odom);

    /// 设置gins的各种配置项，可以在构建时调用，也可以构造完成后，静止初始化结束时调用
    void SetOptions(Options options);

    /**
     * 获取当前状态
     * 如果IMU没有积分，就返回最后优化的状态
     * 如果有，利用IMU积分，预测自己的状态
     * @return
     */
    NavStated GetState() const;

   private:
    // 优化
    void Optimize();

    Options options_;
    double current_time_ = 0.0;  // 当前时间

    std::shared_ptr<IMUPreintegration> pre_integ_ = nullptr;
    std::shared_ptr<NavStated> last_frame_ = nullptr;  // 上一个时刻状态
    std::shared_ptr<NavStated> this_frame_ = nullptr;  // 当前时刻状态
    Matrix<double, 15, 15> prior_info_ = Matrix<double, 15, 15>::Identity() * 1e2;     // 当前时刻先验

    /// 两帧GNSS观测
    GNSS last_gnss_;
    GNSS this_gnss_;

    IMU last_imu_;    // 上时刻IMU
    Odom last_odom_;  // 上时刻odom
    bool last_odom_set_ = false;

    /// 标志位
    bool first_gnss_received_ = false;  // 是否已收到第一个gnss信号
    bool first_imu_received_ = false;   // 是否已收到第一个imu信号
};