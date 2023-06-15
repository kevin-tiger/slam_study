#pragma once
#include "common/basetype.h"
#include "common/math_utils.h"
#include "common/point_types.h"
#include "ndt/ndt_inc.h"

class IncrementalNDTLO 
{
public:
    struct Options 
    {
        Options() {}
        double kf_distance_ = 0.5;            // 关键帧距离
        double kf_angle_deg_ = 30;            // 旋转角度
        bool display_realtime_cloud_ = true;  // 是否显示实时点云
        IncNdt3d::Options ndt3d_options_;     // NDT3D 的配置
    };

    IncrementalNDTLO(Options options = Options()) 
    : options_(options) 
    {
        if (options_.display_realtime_cloud_) {
            // viewer_ = std::make_shared<PCLMapViewer>(0.5);
        }
        ndt_ = IncNdt3d(options_.ndt3d_options_);
    }

    /**
     * 往LO中增加一个点云
     * @param scan  当前帧点云
     * @param pose 估计pose
     */
    void AddCloud(CloudPtr scan, Sophus::SE3d& pose, bool use_guess = false);

private:
    /// 判定是否为关键帧
    bool IsKeyframe(const Sophus::SE3d& current_pose);

   private:
    Options options_;
    bool first_frame_ = true;
    std::vector<Sophus::SE3d> estimated_poses_;  // 所有估计出来的pose，用于记录轨迹和预测下一个帧
    Sophus::SE3d last_kf_pose_;                  // 上一关键帧的位姿
    int cnt_frame_ = 0;

    IncNdt3d ndt_;
    // std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
};