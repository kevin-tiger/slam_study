#include "fusion/fusion.h"

Fusion::Fusion(const std::string& config_yaml) 
{
    config_yaml_ = config_yaml;
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);
    ndt_.setResolution(1.0);
}

bool Fusion::Init() {
    // 地图原点
    auto yaml = YAML::LoadFile(config_yaml_);
    auto origin_data = yaml["origin"].as<std::vector<double>>();
    map_origin_ = Vector3d(origin_data[0], origin_data[1], origin_data[2]);

    // 地图目录
    data_path_ = yaml["map_data"].as<std::string>();
    LoadMapIndex();

    // lidar和IMU消息同步
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup& m) { ProcessMeasurements(m); });
    sync_->Init(config_yaml_);

    // lidar和IMU外参
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    Vector3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
    Matrix3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
    TIL_ = Sophus::SE3d(lidar_R_wrt_IMU, lidar_T_wrt_IMU);

    // ui
    ui_ = std::make_shared<PangolinWindow>();
    ui_->Init();
    ui_->SetCurrentScanSize(50);
    return true;
}

void Fusion::ProcessRTK(GNSSPtr gnss) 
{
    gnss->utm_pose_.translation() -= map_origin_;  // 减掉地图原点
    last_gnss_ = gnss;
}

void Fusion::ProcessMeasurements(const MeasureGroup& meas)
{

}

void Fusion::TryInitIMU()
{

}

void Fusion::Predict()
{

}

void Fusion::Undistort()
{

}

void Fusion::Align()
{

}

bool Fusion::SearchRTK()
{

}

void Fusion::AlignForGrid(Fusion::GridSearchResult& gr)
{

}

bool Fusion::LidarLocalization()
{

}

void Fusion::LoadMap(const Sophus::SE3d& pose)
{

}

void Fusion::LoadMapIndex()
{

}

void Fusion::ProcessIMU(IMUPtr imu)
{

}

void Fusion::ProcessPointCloud(sensor_msgs::PointCloud2::Ptr cloud)
{

}