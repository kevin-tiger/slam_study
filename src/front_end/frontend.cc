#include "front_end/frontend.h"

Frontend::Frontend(const std::string& config_yaml) 
{
     config_yaml_ = config_yaml; 
}

bool Frontend::Init()
{
    cout << "load yaml from " << config_yaml_<< endl;
    auto yaml = YAML::LoadFile(config_yaml_);
    bag_path_ = yaml["bag_path"].as<std::string>();
    lio_yaml_ = yaml["lio_yaml"].as<std::string>();
    cout << "bag_path_ = " << bag_path_ << endl;
    cout << "lio_yaml_ = " << lio_yaml_ << endl;
    system("rm -rf ./data/ch9/*.pcd");
    system("rm -rf ./data/ch9/keyframes.txt");
    LioIEKF::Options options;
    options.with_ui_ = false;  // 跑建图不需要打开前端UI
    lio_ = std::make_shared<LioIEKF>(options);
    lio_->Init(lio_yaml_);
    return true;
}

void Frontend::Run() 
{
    RosbagIO rosbag_io(bag_path_);
    // 先提取RTK pose，注意NCLT只有平移部分
    rosbag_io.AddAutoRTKHandle([this](GNSSPtr gnss) 
    {
        // cout << gnss->unix_time_ << endl;
        gnss_.emplace(gnss->unix_time_, gnss);
        return true;
    });
    rosbag_io.Go();
    RemoveMapOrigin();
    // 再运行LIO
    rosbag_io.AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool 
    {
            lio_->PCLCallBack(cloud);
            ExtractKeyFrame(lio_->GetCurrentState());
            return true;
    });
    rosbag_io.AddImuHandle([&](IMUPtr imu) {
        lio_->IMUCallBack(imu);
        return true;
    });
    rosbag_io.Go();

    // 保存运行结果
    SaveKeyframes();
}

void Frontend::ExtractKeyFrame(const NavStated& state)
{
    if (last_kf_ == nullptr) 
    {
        if (!lio_->GetCurrentScan()) 
        {
            // LIO没完成初始化
            return;
        }
        // 第一个帧
        auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
        FindGPSPose(kf);
        kf->SaveAndUnloadScan("./data/ch9/");
        keyframes_.emplace(kf->id_, kf);
        last_kf_ = kf;
    } 
    else 
    {
        // 计算当前state与kf之间的相对运动阈值
        Sophus::SE3d delta = last_kf_->lidar_pose_.inverse() * state.GetSE3();
        if (delta.translation().norm() > kf_dis_th_ || delta.so3().log().norm() > kf_ang_th_deg_ * math::kDEG2RAD) {
            auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
            FindGPSPose(kf);
            keyframes_.emplace(kf->id_, kf);
            kf->SaveAndUnloadScan("./data/ch9/");
            cout << "生成关键帧" << kf->id_ << endl;
            last_kf_ = kf;
        }
    }
}

void Frontend::FindGPSPose(std::shared_ptr<Keyframe> kf) 
{
    Sophus::SE3d pose;
    GNSSPtr match;
    if (math::PoseInterp<GNSSPtr>(
            kf->timestamp_, gnss_, [](const GNSSPtr& gnss) -> Sophus::SE3d { return gnss->utm_pose_; }, pose, match)) {
        kf->rtk_pose_ = pose;
        kf->rtk_valid_ = true;
    } else {
        kf->rtk_valid_ = false;
    }
}

void Frontend::SaveKeyframes() 
{
    std::ofstream fout("./data/ch9/keyframes.txt");
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

void Frontend::RemoveMapOrigin()
{
    if (gnss_.empty()) 
    {
        return;
    }
    bool origin_set = false;
    for (auto& p : gnss_) 
    {
        if (p.second->status_ == GpsStatusType::GNSS_FIXED_SOLUTION) 
        {
            map_origin_ = p.second->utm_pose_.translation();
            origin_set = true;
            cout << "map origin is set to " << map_origin_.transpose() << endl;
            // auto yaml = YAML::LoadFile(config_yaml_);
            // std::vector<double> ori{map_origin_[0], map_origin_[1], map_origin_[2]};
            // yaml["origin"] = ori;
            // std::ofstream fout(config_yaml_);
            // fout << yaml;
            break;
        }
    }
}