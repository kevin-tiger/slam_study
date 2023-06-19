#include "imu_preinteg/lio_preinteg.h"

LioPreinteg::LioPreinteg(Options options) : options_(options), preinteg_(new IMUPreintegration()) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);

    double bg_rw2 = 1.0 / (options_.bias_gyro_var_ * options_.bias_gyro_var_);
    options_.bg_rw_info_.diagonal() << bg_rw2, bg_rw2, bg_rw2;
    double ba_rw2 = 1.0 / (options_.bias_acce_var_ * options_.bias_acce_var_);
    options_.ba_rw_info_.diagonal() << ba_rw2, ba_rw2, ba_rw2;

    double gp2 = options_.ndt_pos_noise_ * options_.ndt_pos_noise_;
    double ga2 = options_.ndt_ang_noise_ * options_.ndt_ang_noise_;

    options_.ndt_info_.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gp2;

    options_.ndt_options_.nearby_type_ = IncNdt3d::NearbyType::CENTER;
    ndt_ = IncNdt3d(options_.ndt_options_);
}

bool LioPreinteg::Init(const std::string &config_yaml) {
    if (!LoadFromYAML(config_yaml)) {
        cout << "init failed." << endl;
        return false;
    }

    if (options_.with_ui_) {
        ui_ = std::make_shared<PangolinWindow>();
        ui_->Init();
    }

    return true;
}

void LioPreinteg::ProcessMeasurements(const MeasureGroup &meas) 
{
    measures_ = meas;
    if (imu_need_init_) {
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

void LioPreinteg::TryInitIMU()
{
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        // 噪声由初始化器估计
        options_.preinteg_options_.noise_gyro_ = sqrt(imu_init_.GetCovGyro()[0]);
        options_.preinteg_options_.noise_acce_ = sqrt(imu_init_.GetCovAcce()[0]);
        options_.preinteg_options_.init_ba_ = imu_init_.GetInitBa();
        options_.preinteg_options_.init_bg_ = imu_init_.GetInitBg();

        preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        imu_need_init_ = false;

        current_nav_state_.v_.setZero();
        current_nav_state_.bg_ = imu_init_.GetInitBg();
        current_nav_state_.ba_ = imu_init_.GetInitBa();
        current_nav_state_.timestamp_ = measures_.imu_.back()->timestamp_;
        
        last_nav_state_ = current_nav_state_;
        last_imu_ = measures_.imu_.back();

        cout << "IMU初始化成功" << endl;
    }
}

void LioPreinteg::Predict()
{
    imu_states_.clear();
    imu_states_.emplace_back(last_nav_state_);

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        if (last_imu_ != nullptr) {
            preinteg_->Integrate(*imu, imu->timestamp_ - last_imu_->timestamp_);
        }

        last_imu_ = imu;
        imu_states_.emplace_back(preinteg_->Predict(last_nav_state_, imu_init_.GetGravity()));
    }
}

void LioPreinteg::Undistort()
{
    scan_undistort_ = measures_.lidar_;
}

void LioPreinteg::Align()
{
    CloudPtr scan_undistort_trans(new PointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;
    current_scan_ = scan_undistort_;
    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);
    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);
    /// the first scan
    if (flg_first_scan_) {
        ndt_.AddCloud(current_scan_);
        preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
        flg_first_scan_ = false;
        return;
    }
    // 后续的scan，使用NDT配合pose进行更新
    ndt_.SetSource(current_scan_filter);
    current_nav_state_ = preinteg_->Predict(last_nav_state_, imu_init_.GetGravity());
    ndt_pose_ = current_nav_state_.GetSE3();
    ndt_.AlignNdt(ndt_pose_);

    Optimize();

    // 若运动了一定范围，则把点云放入地图中
    Sophus::SE3d current_pose = current_nav_state_.GetSE3();
    Sophus::SE3d delta_pose = last_ndt_pose_.inverse() * current_pose;

    if (delta_pose.translation().norm() > 0.3 || delta_pose.so3().log().norm() > math::deg2rad(5.0)) {
        // 将地图合入NDT中
        CloudPtr current_scan_world(new PointCloudType);
        pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
        ndt_.AddCloud(current_scan_world);
        last_ndt_pose_ = current_pose;
    }
    // 放入UI
    if (ui_) {
        ui_->UpdateScan(current_scan_, current_nav_state_.GetSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(current_nav_state_);
    }
    frame_num_++;
}

bool LioPreinteg::LoadFromYAML(const std::string &yaml_file) {
    // get params from yaml
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });
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

void LioPreinteg::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) { sync_->ProcessCloud(msg); }

void LioPreinteg::IMUCallBack(IMUPtr msg_in) { sync_->ProcessIMU(msg_in); }

void LioPreinteg::Optimize() 
{
    // 调用g2o求解优化问题
    // 上一个state到本时刻state的预积分因子，本时刻的NDT因子

    /// NOTE 这些东西是对参数非常敏感的。相差几个数量级的话，容易出现优化不动的情况

    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 上时刻顶点， pose, v, bg, ba
    auto v0_pose = new VertexPose();
    v0_pose->setId(0);
    v0_pose->setEstimate(last_nav_state_.GetSE3());
    optimizer.addVertex(v0_pose);

    auto v0_vel = new VertexVelocity();
    v0_vel->setId(1);
    v0_vel->setEstimate(last_nav_state_.v_);
    optimizer.addVertex(v0_vel);

    auto v0_bg = new VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate(last_nav_state_.bg_);
    optimizer.addVertex(v0_bg);

    auto v0_ba = new VertexAccBias();
    v0_ba->setId(3);
    v0_ba->setEstimate(last_nav_state_.ba_);
    optimizer.addVertex(v0_ba);

    // 本时刻顶点，pose, v, bg, ba
    auto v1_pose = new VertexPose();
    v1_pose->setId(4);
    v1_pose->setEstimate(ndt_pose_);  // NDT pose作为初值
    // v1_pose->setEstimate(current_nav_state_.GetSE3());  // 预测的pose作为初值
    optimizer.addVertex(v1_pose);

    auto v1_vel = new VertexVelocity();
    v1_vel->setId(5);
    v1_vel->setEstimate(current_nav_state_.v_);
    optimizer.addVertex(v1_vel);

    auto v1_bg = new VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate(current_nav_state_.bg_);
    optimizer.addVertex(v1_bg);

    auto v1_ba = new VertexAccBias();
    v1_ba->setId(7);
    v1_ba->setEstimate(current_nav_state_.ba_);
    optimizer.addVertex(v1_ba);

    // imu factor
    auto edge_inertial = new EdgeInertial(preinteg_, imu_init_.GetGravity());
    edge_inertial->setVertex(0, v0_pose);
    edge_inertial->setVertex(1, v0_vel);
    edge_inertial->setVertex(2, v0_bg);
    edge_inertial->setVertex(3, v0_ba);
    edge_inertial->setVertex(4, v1_pose);
    edge_inertial->setVertex(5, v1_vel);
    auto *rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    edge_inertial->setRobustKernel(rk);
    optimizer.addEdge(edge_inertial);

    // 零偏随机游走
    auto *edge_gyro_rw = new EdgeGyroRW();
    edge_gyro_rw->setVertex(0, v0_bg);
    edge_gyro_rw->setVertex(1, v1_bg);
    edge_gyro_rw->setInformation(options_.bg_rw_info_);
    optimizer.addEdge(edge_gyro_rw);

    auto *edge_acc_rw = new EdgeAccRW();
    edge_acc_rw->setVertex(0, v0_ba);
    edge_acc_rw->setVertex(1, v1_ba);
    edge_acc_rw->setInformation(options_.ba_rw_info_);
    optimizer.addEdge(edge_acc_rw);

    // 上一帧pose, vel, bg, ba的先验
    auto *edge_prior = new EdgePriorPoseNavState(last_nav_state_, prior_info_);
    edge_prior->setVertex(0, v0_pose);
    edge_prior->setVertex(1, v0_vel);
    edge_prior->setVertex(2, v0_bg);
    edge_prior->setVertex(3, v0_ba);
    optimizer.addEdge(edge_prior);

    /// 使用NDT的pose进行观测
    auto *edge_ndt = new EdgeGNSS(v1_pose, ndt_pose_);
    edge_ndt->setInformation(options_.ndt_info_);
    optimizer.addEdge(edge_ndt);

    v0_bg->setFixed(true);
    v0_ba->setFixed(true);

    // go
    optimizer.setVerbose(options_.verbose_);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // get results
    last_nav_state_.R_ = v0_pose->estimate().so3();
    last_nav_state_.p_ = v0_pose->estimate().translation();
    last_nav_state_.v_ = v0_vel->estimate();
    last_nav_state_.bg_ = v0_bg->estimate();
    last_nav_state_.ba_ = v0_ba->estimate();

    current_nav_state_.R_ = v1_pose->estimate().so3();
    current_nav_state_.p_ = v1_pose->estimate().translation();
    current_nav_state_.v_ = v1_vel->estimate();
    current_nav_state_.bg_ = v1_bg->estimate();
    current_nav_state_.ba_ = v1_ba->estimate();

    /// 重置预积分

    options_.preinteg_options_.init_bg_ = current_nav_state_.bg_;
    options_.preinteg_options_.init_ba_ = current_nav_state_.ba_;
    preinteg_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);

    // 计算当前时刻先验
    // 构建hessian
    // 15x2，顺序：v0_pose, v0_vel, v0_bg, v0_ba, v1_pose, v1_vel, v1_bg, v1_ba
    //            0       6        9     12     15        21      24     27
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    H.block<24, 24>(0, 0) += edge_inertial->GetHessian();

    Eigen::Matrix<double, 6, 6> Hgr = edge_gyro_rw->GetHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    Eigen::Matrix<double, 6, 6> Har = edge_acc_rw->GetHessian();
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    H.block<15, 15>(0, 0) += edge_prior->GetHessian();
    H.block<6, 6>(15, 15) += edge_ndt->GetHessian();

    H = math::Marginalize(H, 0, 14);
    prior_info_ = H.block<15, 15>(15, 15);

    NormalizeVelocity();
    last_nav_state_ = current_nav_state_;
}

void LioPreinteg::NormalizeVelocity() {
    /// 限制v的变化
    /// 一般是-y 方向速度
    Vector3d v_body = current_nav_state_.R_.inverse() * current_nav_state_.v_;
    if (v_body[1] > 0) {
        v_body[1] = 0;
    }
    v_body[2] = 0;

    if (v_body[1] < -2.0) {
        v_body[1] = -2.0;
    }

    if (v_body[0] > 0.1) {
        v_body[0] = 0.1;
    } else if (v_body[0] < -0.1) {
        v_body[0] = -0.1;
    }

    current_nav_state_.v_ = current_nav_state_.R_ * v_body;
}
