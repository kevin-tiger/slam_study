#include "fusion/fusion.h"

Fusion::Fusion(const std::string& config_yaml) 
{
    config_yaml_ = config_yaml;
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;  // 本节数据不需要轮速计
    imu_init_ = StaticIMUInit(imu_init_options);
    ndt_.setResolution(1.0);
}

bool Fusion::Init() 
{
    // 地图原点
    auto yaml = YAML::LoadFile(config_yaml_);
    auto origin_data = yaml["origin"].as<std::vector<double>>();
    map_origin_ = Vector3d(origin_data[0], origin_data[1], origin_data[2]);
    cout << fixed << setprecision(3) << "map_origin_ = " << map_origin_.transpose() << endl;

    // 地图目录
    data_path_ = yaml["map_data"].as<std::string>();
    cout << "data_path_ = " << data_path_ << endl;
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
    cout << fixed << setprecision(3) << "TIL_ = \n" << TIL_.matrix() << endl;
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
    // cout << "ProcessMeasurements" << endl;
    measures_ = meas;

    if (imu_need_init_) 
    {
        TryInitIMU();
        return;
    }

    /// 以下三步与LIO一致，只是align完成地图匹配工作
    if (status_ == Status::WORKING) 
    {
        Predict();
        Undistort();
    } 
    else 
    {
        scan_undistort_ = measures_.lidar_;
    }

    Align();
}

void Fusion::TryInitIMU()
{
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        ESKFD::Options options;
        // 噪声由初始化器估计
        // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        options.update_bias_acce_ = false;
        options.update_bias_gyro_ = false;
        eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        cout << "IMU初始化成功" << endl;
    }
}

void Fusion::Predict()
{
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());

    /// 对IMU状态进行预测
    for (auto& imu : measures_.imu_) {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
    }
}

void Fusion::Undistort()
{
    scan_undistort_ = measures_.lidar_;
}

void Fusion::Align()
{
    CloudPtr scan_undistort_trans(new PointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
    scan_undistort_ = scan_undistort_trans;
    current_scan_ = scan_undistort_;
    current_scan_ = VoxelCloud(current_scan_, 0.5);

    if (status_ == Status::WAITING_FOR_RTK) 
    {
        // 若存在最近的RTK信号，则尝试初始化
        if (last_gnss_ != nullptr) 
        {
            if (SearchRTK()) 
            {
                status_ == Status::WORKING;
                ui_->UpdateScan(current_scan_, eskf_.GetNominalSE3());
                ui_->UpdateNavState(eskf_.GetNominalState());
            }
        }
    } 
    else 
    {
        LidarLocalization();
        ui_->UpdateScan(current_scan_, eskf_.GetNominalSE3());
        ui_->UpdateNavState(eskf_.GetNominalState());
    }
}

bool Fusion::SearchRTK()
{
    if (init_has_failed_) 
    {
        if ((last_gnss_->utm_pose_.translation() - last_searched_pos_.translation()).norm() < 20.0) 
        {
            cout << "skip this position" << endl;
            return false;
        }
    }
    // 由于RTK不带姿态，我们必须先搜索一定的角度范围
    std::vector<GridSearchResult> search_poses;
    LoadMap(last_gnss_->utm_pose_);

    /// 由于RTK不带角度，这里按固定步长扫描RTK角度
    double grid_ang_range = 360.0, grid_ang_step = 10;  // 角度搜索范围与步长
    for (double ang = 0; ang < grid_ang_range; ang += grid_ang_step) {
        Sophus::SE3d pose(Sophus::SO3d::rotZ(ang * math::kDEG2RAD), Vector3d(0, 0, 0) + last_gnss_->utm_pose_.translation());
        GridSearchResult gr;
        gr.pose_ = pose;
        search_poses.emplace_back(gr);
    }

    cout << "grid search poses: " << search_poses.size() << endl;
    // std::for_each(std::execution::par_unseq, search_poses.begin(), search_poses.end(),
    //               [this](GridSearchResult& gr) { AlignForGrid(gr); });
    std::for_each(search_poses.begin(), search_poses.end(),
    [this](GridSearchResult& gr) 
    { 
        AlignForGrid(gr); 
    });

    // 选择最优的匹配结果
    auto max_ele = std::max_element(search_poses.begin(), search_poses.end(),
                                    [](const auto& g1, const auto& g2) { return g1.score_ < g2.score_; });
    cout << "max score: " << max_ele->score_ << ", pose: \n" << max_ele->result_pose_.matrix() << endl;
    if (max_ele->score_ > rtk_search_min_score_) 
    {
        cout << "初始化成功, score: " << max_ele->score_ << ">" << rtk_search_min_score_ << endl;
        status_ = Status::WORKING;

        /// 重置滤波器状态
        auto state = eskf_.GetNominalState();
        state.R_ = max_ele->result_pose_.so3();
        state.p_ = max_ele->result_pose_.translation();
        state.v_.setZero();
        eskf_.SetX(state, eskf_.GetGravity());

        ESKFD::Mat18T cov;
        cov = ESKFD::Mat18T::Identity() * 1e-4;
        cov.block<12, 12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity() * 1e-6;
        eskf_.SetCov(cov);

        return true;
    }

    init_has_failed_ = true;
    last_searched_pos_ = last_gnss_->utm_pose_;
    return false;
}

void Fusion::AlignForGrid(Fusion::GridSearchResult& gr)
{
    /// 多分辨率
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(40);

    ndt.setInputSource(current_scan_);
    auto map = ref_cloud_;

    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 4.0, 3.0};
    Matrix4f T = gr.pose_.matrix().cast<float>();
    for (auto& r : res) {
        auto rough_map = VoxelCloud(map, r * 0.1);
        ndt.setInputTarget(rough_map);
        ndt.setResolution(r);
        ndt.align(*output, T);
        T = ndt.getFinalTransformation();
    }

    gr.score_ = ndt.getTransformationProbability();
    gr.result_pose_ = math::Mat4ToSE3(ndt.getFinalTransformation());
}

bool Fusion::LidarLocalization()
{
    Sophus::SE3d pred = eskf_.GetNominalSE3();
    LoadMap(pred);

    ndt_.setInputCloud(current_scan_);
    CloudPtr output(new PointCloudType);
    ndt_.align(*output, pred.matrix().cast<float>());

    Sophus::SE3d pose = math::Mat4ToSE3(ndt_.getFinalTransformation());
    eskf_.ObserveSE3(pose, 1e-1, 1e-2);

    cout << "lidar loc score: " << ndt_.getTransformationProbability() << endl;

    return true;
}

void Fusion::LoadMap(const Sophus::SE3d& pose)
{
    int gx = floor((pose.translation().x() - 50.0) / 100);
    int gy = floor((pose.translation().y() - 50.0) / 100);
    Vector2i key(gx, gy);

    // 一个区域的周边地图，我们认为9个就够了
    std::set<Vector2i, math::less_vec<2>> surrounding_index{
        key + Vector2i(0, 0), key + Vector2i(-1, 0), key + Vector2i(-1, -1), key + Vector2i(-1, 1), key + Vector2i(0, -1),
        key + Vector2i(0, 1), key + Vector2i(1, 0),  key + Vector2i(1, -1),  key + Vector2i(1, 1),
    };
    // 加载必要区域
    bool map_data_changed = false;
    int cnt_new_loaded = 0, cnt_unload = 0;
    for (auto& k : surrounding_index) 
    {
        if (map_data_index_.find(k) == map_data_index_.end()) 
        {
            // 该地图数据不存在
            continue;
        }
        if (map_data_.find(k) == map_data_.end()) 
        {
            // 加载这个区块
            CloudPtr cloud(new PointCloudType);
            pcl::io::loadPCDFile(data_path_ + std::to_string(k[0]) + "_" + std::to_string(k[1]) + ".pcd", *cloud);
            map_data_.emplace(k, cloud);
            map_data_changed = true;
            cnt_new_loaded++;
        }
    }
    // 卸载不需要的区域，这个稍微加大一点，不需要频繁卸载
    for (auto iter = map_data_.begin(); iter != map_data_.end();) 
    {
        if ((iter->first - key).cast<float>().norm() > 3.0) 
        {
            // 卸载本区块
            iter = map_data_.erase(iter);
            cnt_unload++;
            map_data_changed = true;
        } 
        else 
        {
            iter++;
        }
    }
    cout << "new loaded: " << cnt_new_loaded << ", unload: " << cnt_unload << endl;
    if (map_data_changed) 
    {
        // rebuild ndt target map
        ref_cloud_.reset(new PointCloudType);
        for (auto& mp : map_data_) 
        {
            *ref_cloud_ += *mp.second;
        }
        cout << "rebuild global cloud, grids: " << map_data_.size() << endl;
        ndt_.setInputTarget(ref_cloud_);
    }
    ui_->UpdatePointCloudGlobal(map_data_);
}

void Fusion::LoadMapIndex()
{
    std::ifstream fin(data_path_ + "/map_index.txt");
    while (!fin.eof()) {
        int x, y;
        fin >> x >> y;
        map_data_index_.emplace(Vector2i(x, y));
    }
    cout << "map_data_index_.size() = " << map_data_index_.size() << endl;
    fin.close();
}

void Fusion::ProcessIMU(IMUPtr imu)
{
    sync_->ProcessIMU(imu);
}

void Fusion::ProcessPointCloud(sensor_msgs::PointCloud2::Ptr cloud)
{
    sync_->ProcessCloud(cloud);
}