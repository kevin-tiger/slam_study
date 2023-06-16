#include "eskf/lio_eskf.h"

LooselyLIO::LooselyLIO(Options options) 
: options_(options) 
{
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);
}

bool LooselyLIO::Init(const std::string &config_yaml) 
{
    /// 初始化自身的参数
    if (!LoadFromYAML(config_yaml)) 
    {
        return false;
    }
    /// 初始化NDT LO的参数
    IncrementalNDTLO::Options indt_options;
    indt_options.display_realtime_cloud_ = false;  // 这个程序自己有UI，不用PCL中的
    inc_ndt_lo_ = std::make_shared<IncrementalNDTLO>(indt_options);
    /// 初始化UI
    if (options_.with_ui_) 
    {
        ui_ = std::make_shared<PangolinWindow>();
        ui_->Init();
    }
    return true;
}

bool LooselyLIO::LoadFromYAML(const std::string &yaml_file) 
{
    // get params from yaml
    sync_ = std::make_shared<MessageSync>(
    [this](const MeasureGroup &m) 
    { 
        ProcessMeasurements(m); 
    });
    sync_->Init(yaml_file);
    /// 自身参数主要是雷达与IMU外参
    auto yaml = YAML::LoadFile(yaml_file);
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    Vector3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
    Matrix3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
    TIL_ = Sophus::SE3d(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    return true;
}

void LooselyLIO::ProcessMeasurements(const MeasureGroup &meas) 
{
    // cout << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size() << endl;
    measures_ = meas;
    if (imu_need_init_) 
    {
        // 初始化IMU系统
        TryInitIMU();
        return;
    }
    // 利用IMU数据进行状态预测
    Predict();
    // 对点云去畸变
    Undistort();
    // 配准
    Align();
}

void LooselyLIO::Predict() 
{
    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) 
    {
        eskf_.Predict(*imu);
    }
}

void LooselyLIO::Undistort() 
{
    scan_undistort_ = measures_.lidar_;
}

void LooselyLIO::Align() 
{
    CloudPtr scan_undistort_trans(new PointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
    scan_undistort_ = scan_undistort_trans;
    auto current_scan = scan_undistort_;
    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan);
    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);
    /// 处理首帧雷达数据
    if (flg_first_scan_) 
    {
        Sophus::SE3d pose;
        inc_ndt_lo_->AddCloud(current_scan_filter, pose);
        flg_first_scan_ = false;
        return;
    }
    /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
    Sophus::SE3d pose_predict = eskf_.GetNominalSE3();
    inc_ndt_lo_->AddCloud(current_scan_filter, pose_predict, true);
    pose_of_lo_ = pose_predict;
    eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);
    if (options_.with_ui_) 
    {
        // 放入UI
        ui_->UpdateScan(current_scan, eskf_.GetNominalSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(eskf_.GetNominalState());
    }
    frame_num_++;
}

void LooselyLIO::TryInitIMU() 
{
    for (auto imu : measures_.imu_) 
    {
        imu_init_.AddIMU(*imu);
    }
    if (imu_init_.InitSuccess()) 
    {
        // 读取初始零偏，设置ESKF
        ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;
        cout << "IMU初始化成功" << endl;
    }
}

void LooselyLIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
     sync_->ProcessCloud(msg); 
}

void LooselyLIO::IMUCallBack(IMUPtr msg_in) 
{
     sync_->ProcessIMU(msg_in); 
}