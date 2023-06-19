#pragma once
#include "common/basetype.h"
#include "common/nav_state.h"
#include "sensor/imu.h"

/**
 * IMU 预积分器
 *
 * 调用Integrate来插入新的IMU读数，然后通过Get函数得到预积分的值
 * 雅可比也可以通过本类获得，可用于构建g2o的边类
 */
class IMUPreintegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// 参数配置项
    /// 初始的零偏需要设置，其他可以不改
    struct Options {
        Options() {}
        Vector3d init_bg_ = Vector3d::Zero();  // 初始零偏
        Vector3d init_ba_ = Vector3d::Zero();  // 初始零偏
        double noise_gyro_ = 1e-2;       // 陀螺噪声，标准差
        double noise_acce_ = 1e-1;       // 加计噪声，标准差
    };

    IMUPreintegration(Options options = Options());

    /**
     * 插入新的IMU数据
     * @param imu   imu 读数
     * @param dt    时间差
     */
    void Integrate(const IMU &imu, double dt);

    /**
     * 从某个起始点开始预测积分之后的状态
     * @param start 起始时时刻状态
     * @return  预测的状态
     */
    NavStated Predict(const NavStated &start, const Vector3d &grav = Vector3d(0, 0, -9.81)) const;

    /// 获取修正之后的观测量，bias可以与预积分时期的不同，会有一阶修正
    Sophus::SO3d GetDeltaRotation(const Vector3d &bg);
    Vector3d GetDeltaVelocity(const Vector3d &bg, const Vector3d &ba);
    Vector3d GetDeltaPosition(const Vector3d &bg, const Vector3d &ba);

   public:
    double dt_ = 0;                          // 整体预积分时间
    Matrix<double, 9, 9> cov_ = Matrix<double, 9, 9>::Zero();              // 累计噪声矩阵
    Matrix<double, 6, 6> noise_gyro_acce_ = Matrix<double, 6, 6>::Zero();  // 测量噪声矩阵

    // 零偏
    Vector3d bg_ = Vector3d::Zero();
    Vector3d ba_ = Vector3d::Zero();

    // 预积分观测量
    Sophus::SO3d dR_;
    Vector3d dv_ = Vector3d::Zero();
    Vector3d dp_ = Vector3d::Zero();

    // 雅可比矩阵
    Matrix3d dR_dbg_ = Matrix3d::Zero();
    Matrix3d dV_dbg_ = Matrix3d::Zero();
    Matrix3d dV_dba_ = Matrix3d::Zero();
    Matrix3d dP_dbg_ = Matrix3d::Zero();
    Matrix3d dP_dba_ = Matrix3d::Zero();
};
