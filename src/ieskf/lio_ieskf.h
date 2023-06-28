#pragma once
#include "common/basetype.h"
#include "ieskf/ieskf.hpp"
#include "init/static_imu_init.h"
#include "sync/measure_sync.h"
#include "common/point_cloud_utils.h"
#include "ndt/ndt_inc.h"
#include "viewer/pangolin_window.h"

class LioIEKF 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct Options 
    {
        Options() {}
        bool save_motion_undistortion_pcd_ = true;  // 是否保存去畸变前后的点云
        bool with_ui_ = true;                        // 是否带着UI
    };
    LioIEKF(Options options = Options());
    ~LioIEKF() = default;
    /// init without ros
    bool Init(const std::string& config_yaml);
    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);
    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);
    /// 结束程序，退出UI
    void Finish();
    /// 获取当前姿态
    NavStated GetCurrentState() const { return ieskf_.GetNominalState(); }
    /// 获取当前扫描
    CloudPtr GetCurrentScan() const { return current_scan_; }
private:
    bool LoadFromYAML(const std::string& yaml_file);
    /// 尝试让IMU初始化
    void TryInitIMU();
    /// 利用IMU预测状态信息
    void Predict();
    /// 对measures_中的点云去畸变
    void Undistort();
    /// 执行一次配准和观测
    void Align();
private:
    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;
    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;
    /// NDT数据
    IncNdt3d ndt_;
    Sophus::SE3d last_pose_;
    // flags
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;
    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan
    IESKFD ieskf_;  // IESKF
    Sophus::SE3d TIL_;       // Lidar与IMU之间外参
    Options options_;
    std::shared_ptr<PangolinWindow> ui_ = nullptr;
};