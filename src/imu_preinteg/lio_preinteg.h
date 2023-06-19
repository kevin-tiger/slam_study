#pragma once
#include "common/basetype.h"
#include "imu_preinteg/imu_preintegration.h"
#include "ndt/ndt_inc.h"
#include "viewer/pangolin_window.h"
#include "sync/measure_sync.h"
#include "init/static_imu_init.h"
#include "g2o_opti/g2o_types.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "g2o/core/robust_kernel_impl.h"

/**
 * 第8章 基于预积分系统的LIO
 * 框架与前文一致，但之前由IEKF处理的部分变为预积分优化
 */
class LioPreinteg {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Options {
        Options() {}
        bool with_ui_ = true;  // 是否带着UI
        bool verbose_ = true;  // 打印调试信息

        double bias_gyro_var_ = 1e-2;           // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-2;           // 加计零偏游走标准差
        Matrix3d bg_rw_info_ = Matrix3d::Identity();  // 陀螺随机游走信息阵
        Matrix3d ba_rw_info_ = Matrix3d::Identity();  // 加计随机游走信息阵

        double ndt_pos_noise_ = 0.1;                   // NDT位置方差
        double ndt_ang_noise_ = 2.0 * math::kDEG2RAD;  // NDT角度方差
        Matrix<double, 6, 6> ndt_info_ = Matrix<double, 6, 6>::Identity();           // 6D NDT 信息矩阵

        IMUPreintegration::Options preinteg_options_;  // 预积分参数
        IncNdt3d::Options ndt_options_;                     // NDT 参数
    };

    LioPreinteg(Options options = Options());
    ~LioPreinteg() = default;

    /// init without ros
    bool Init(const std::string& config_yaml);
    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);
    /// 结束程序，退出UI
    void Finish();
   private:
    bool LoadFromYAML(const std::string& yaml_file);
    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);
    /// 尝试让IMU初始化
    void TryInitIMU();
    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();
    /// 对measures_中的点云去畸变
    void Undistort();
    /// 执行一次配准和观测
    void Align();
    /// 执行预积分+NDT pose优化
    void Optimize();
    /// 将速度限制在正常区间
    void NormalizeVelocity();
    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;
    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;
    // optimize相关
    NavStated last_nav_state_, current_nav_state_;  // 上一时刻状态与本时刻状态
    Matrix<double, 15, 15> prior_info_ = Matrix<double, 15, 15>::Identity();        // 先验约束
    std::shared_ptr<IMUPreintegration> preinteg_ = nullptr;
    IMUPtr last_imu_ = nullptr;
    /// NDT数据
    IncNdt3d ndt_;
    Sophus::SE3d ndt_pose_;
    Sophus::SE3d last_ndt_pose_;
    // flags
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;
    MeasureGroup measures_;  // sync IMU and lidar scan
    std::vector<NavStated> imu_states_;
    Sophus::SE3d TIL_;  // Lidar与IMU之间外参
    Options options_;
    std::shared_ptr<PangolinWindow> ui_ = nullptr;
};