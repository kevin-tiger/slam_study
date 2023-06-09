#pragma once
#include "common/basetype.h"
#include "common/nav_state.h"
#include "common/math_utils.h"
#include "sensor/gnss.h"
#include "pangolin_window_impl.h"
#include "common/point_types.h"

class PangolinWindow 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PangolinWindow();
    ~PangolinWindow();
    /// @brief 初始化窗口，后台启动render线程。
    /// @note 与opengl/pangolin无关的初始化，尽量放到此函数体中;
    ///       opengl/pangolin相关的内容，尽量放到PangolinWindowImpl::Init中。
    bool Init();

    /// 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
    void UpdatePointCloudGlobal(const std::map<Vector2i, CloudPtr, math::less_vec<2>>& cloud);

    /// 更新kalman滤波器状态
    void UpdateNavState(const NavStated& state);

    /// 更新一次scan和它对应的Pose
    void UpdateScan(CloudPtr cloud, const Sophus::SE3d& pose);

    /// 更新GPS定位结果
    void UpdateGps(const GNSS& gps);

    /// 等待显示线程结束，并释放资源
    void Quit();

    /// 用户是否已经退出UI
    bool ShouldQuit();

    /// 设置IMU到雷达的外参
    void SetTImuLidar(const Sophus::SE3d& T_imu_lidar);

    /// 设置需要保留多少个扫描数据
    void SetCurrentScanSize(int current_scan_size);

private:
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};