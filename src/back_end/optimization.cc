#include "back_end/optimization.h"

Optimization::Optimization(const std::string& yaml) 
{
     yaml_ = yaml; 
}

bool Optimization::Init(int stage) 
{
    stage_ = stage;
    if (!LoadKeyFrames("./data/ch9/keyframes.txt", keyframes_)) 
    {
        cout << "cannot load keyframes.txt" << endl;
        return false;
    }
    cout << "keyframes: " << keyframes_.size() << endl;
    // 读参数
    auto yaml = YAML::LoadFile(yaml_);
    rtk_outlier_th_ = yaml["rtk_outlier_th"].as<double>();
    lidar_continuous_num_ = yaml["lidar_continuous_num"].as<int>();
    rtk_has_rot_ = yaml["rtk_has_rot"].as<bool>();
    rtk_pos_noise_ = yaml["rtk_pos_noise"].as<double>();
    rtk_ang_noise_ = yaml["rtk_ang_noise"].as<double>() * math::kDEG2RAD;
    rtk_height_noise_ratio_ = yaml["rtk_height_noise_ratio"].as<double>();
    std::vector<double> rtk_ext_t = yaml["rtk_ext"]["t"].as<std::vector<double>>();
    TBG_ = Sophus::SE3d(Sophus::SO3d(), Vector3d(rtk_ext_t[0], rtk_ext_t[1], rtk_ext_t[2]));
    cout << "TBG = \n" << TBG_.matrix() << endl;
    LoadLoopCandidates();
    return true;
}

void Optimization::Run() 
{
    cout << "running optimization on stage " << stage_ << endl;
    if (!rtk_has_rot_ && stage_ == 1) 
    {
        InitialAlign();
    }
    BuildProblem();  // 建立问题
    SaveG2O("./data/ch9/before.g2o");
    Solve();           // 带着RK求解一遍
    RemoveOutliers();  // 移除异常值
    Solve();           // 再求解一遍
    SaveG2O("./data/ch9/after.g2o");
    SaveResults();  // 保存结果
}

void Optimization::SaveG2O(const std::string& file_name) {
    std::ofstream fout(file_name);
    for (auto& v : vertices_) {
        v.second->write(fout);
    }

    for (auto& e : lidar_edge_) {
        e->write(fout);
    }
    for (auto& e : loop_edge_) {
        e->write(fout);
    }
    fout.close();
}

void Optimization::BuildProblem() 
{
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer_.setAlgorithm(solver);

    AddVertices();
    AddRTKEdges();
    AddLidarEdges();
    AddLoopEdges();
}

void Optimization::AddVertices() 
{
    for (auto& kfp : keyframes_) 
    {
        auto kf = kfp.second;
        // make g2o vertex for this kf
        auto v = new VertexPose();
        v->setId(kf->id_);
        if (stage_ == 1) {
            v->setEstimate(kf->lidar_pose_);
        } else {
            v->setEstimate(kf->opti_pose_1_);
        }
        optimizer_.addVertex(v);
        vertices_.emplace(kf->id_, v);
    }
    cout << "vertex: " << vertices_.size() << endl;
}

void Optimization::AddRTKEdges() 
{
    /// RTK 噪声设置
    Matrix3d info_pos = Matrix3d::Identity() * 1.0 / (rtk_pos_noise_ * rtk_pos_noise_);
    info_pos(2, 2) = 1.0 / (rtk_height_noise_ratio_ * rtk_pos_noise_ * rtk_height_noise_ratio_ * rtk_pos_noise_);
    Matrix3d info_ang = Matrix3d::Identity() * 1.0 / (rtk_ang_noise_ * rtk_ang_noise_);
    Matrix<double, 6, 6> info_all = Matrix<double, 6, 6>::Identity();
    info_all.block<3, 3>(0, 0) = info_pos;
    info_all.block<3, 3>(3, 3) = info_ang;

    cout << "Info of rtk trans: " << info_pos.diagonal().transpose() << endl;

    if (stage_ == 2) {
        info_pos *= 0.01;
        info_all *= 0.01;
    }

    for (auto& kfp : keyframes_) {
        auto kf = kfp.second;
        if (!kf->rtk_valid_) {
            continue;
        }

        if (kf->rtk_heading_valid_) {
            auto edge = new EdgeGNSS(vertices_.at(kf->id_), kf->rtk_pose_);
            edge->setInformation(info_all);
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(rtk_outlier_th_);
            edge->setRobustKernel(rk);
            optimizer_.addEdge(edge);
            gnss_edge_.emplace_back(edge);
        } else {
            auto edge = new EdgeGNSSTransOnly(vertices_.at(kf->id_), kf->rtk_pose_.translation(), TBG_);
            edge->setInformation(info_pos);
            auto rk = new g2o::RobustKernelCauchy();
            rk->setDelta(rtk_outlier_th_);
            edge->setRobustKernel(rk);
            optimizer_.addEdge(edge);
            gnss_trans_edge_.emplace_back(edge);
        }
    }

    cout << "gnss edges: " << gnss_edge_.size() << ", " << gnss_trans_edge_.size() << endl;
}

void Optimization::AddLidarEdges() {
    const double lidar_pos_noise = 0.01, lidar_ang_noise = 0.1 * math::kDEG2RAD;  // RTK 观测的噪声
    Matrix3d info_pos = Matrix3d::Identity() * 1.0 / (lidar_pos_noise * lidar_pos_noise);
    Matrix3d info_ang = Matrix3d::Identity() * 1.0 / (lidar_ang_noise * lidar_ang_noise);
    Matrix<double, 6, 6> info_all = Matrix<double, 6, 6>::Identity();
    info_all.block<3, 3>(0, 0) = info_pos;
    info_all.block<3, 3>(3, 3) = info_ang;

    for (auto iter = keyframes_.begin(); iter != keyframes_.end(); ++iter) {
        auto iter_next = iter;
        for (int i = 0; i < lidar_continuous_num_; ++i) {
            iter_next++;

            if (iter_next == keyframes_.end()) {
                break;
            }

            // 添加iter和iter_next之间的相邻运动
            auto edge = new EdgeRelativeMotion(vertices_.at(iter->second->id_), vertices_.at(iter_next->second->id_),
                                               iter->second->lidar_pose_.inverse() * iter_next->second->lidar_pose_);
            edge->setInformation(info_all);
            optimizer_.addEdge(edge);
            lidar_edge_.emplace_back(edge);
        }
    }
    cout << "lidar edges: " << lidar_edge_.size() << endl;
}

void Optimization::AddLoopEdges() {
    const double loop_pos_noise = 0.1, loop_ang_noise = 0.5 * math::kDEG2RAD;  // RTK 观测的噪声
    Matrix3d info_pos = Matrix3d::Identity() * 1.0 / (loop_pos_noise * loop_pos_noise);
    Matrix3d info_ang = Matrix3d::Identity() * 1.0 / (loop_ang_noise * loop_ang_noise);
    Matrix<double, 6, 6> info_all = Matrix<double, 6, 6>::Identity();
    info_all.block<3, 3>(0, 0) = info_pos;
    info_all.block<3, 3>(3, 3) = info_ang;

    const double loop_rk_th = 5.2;

    for (const auto& lc : loop_candidates_) {
        auto edge = new EdgeRelativeMotion(vertices_.at(lc.idx1_), vertices_.at(lc.idx2_), lc.Tij_);
        edge->setInformation(info_all);
        auto rk = new g2o::RobustKernelCauchy();
        rk->setDelta(loop_rk_th);
        edge->setRobustKernel(rk);
        optimizer_.addEdge(edge);
        loop_edge_.emplace_back(edge);
    }
}

void Optimization::Solve() 
{
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization(0);
    optimizer_.optimize(100);
}

void Optimization::RemoveOutliers() 
{
    // 主要用于移除GNSS的异常值
    int cnt_outlier_removed = 0;
    auto remove_outlier = [&cnt_outlier_removed](g2o::OptimizableGraph::Edge* e) {
        if (e->chi2() > e->robustKernel()->delta()) {
            e->setLevel(1);
            cnt_outlier_removed++;
        } else {
            e->setRobustKernel(nullptr);
        }
    };

    std::for_each(gnss_edge_.begin(), gnss_edge_.end(), remove_outlier);
    std::for_each(gnss_trans_edge_.begin(), gnss_trans_edge_.end(), remove_outlier);
    cout << "gnss outlier: " << cnt_outlier_removed << "/" << gnss_edge_.size() + gnss_trans_edge_.size() << endl;

    cnt_outlier_removed = 0;
    std::for_each(loop_edge_.begin(), loop_edge_.end(), remove_outlier);
    cout << "loop outlier: " << cnt_outlier_removed << "/" << loop_edge_.size() << endl;
}

void Optimization::SaveResults() 
{
    for (auto& v : vertices_) {
        if (stage_ == 1) {
            keyframes_.at(v.first)->opti_pose_1_ = v.second->estimate();
        } else {
            keyframes_.at(v.first)->opti_pose_2_ = v.second->estimate();
        }
    }

    // 比较优化pose和rtk pose
    std::vector<double> rtk_trans_error;
    for (auto& kfp : keyframes_) {
        auto kf = kfp.second;
        Vector3d tWG = kf->rtk_pose_.translation();
        Vector3d t_opti = (kf->opti_pose_1_ * TBG_).translation();
        double n = (tWG - t_opti).head<2>().norm();
        rtk_trans_error.emplace_back(n);
    }

    std::sort(rtk_trans_error.begin(), rtk_trans_error.end());
    cout << "med error: " << rtk_trans_error[rtk_trans_error.size() / 2] << endl;

    // 写入文件
    system("rm ./data/ch9/keyframes.txt");
    std::ofstream fout("./data/ch9/keyframes.txt");
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

void Optimization::InitialAlign() 
{
    // should be p1 = R*p2 + t
    std::vector<Vector3d> pts1, pts2;
    for (auto& kfp : keyframes_) {
        pts1.emplace_back(kfp.second->rtk_pose_.translation());
        pts2.emplace_back(kfp.second->lidar_pose_.translation());
    }

    Vector3d p1, p2;  // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = p1 / N;
    p2 = p2 / N;

    cout  << "p1: " << p1.transpose() << ", p2: " << p2.transpose() << endl;

    std::vector<Vector3d> q1(N), q2(N);  // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Matrix3d W = Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += q1[i] * q2[i].transpose();
    }

    // SVD on W
    Eigen::JacobiSVD<Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Matrix3d R = U * (V.transpose());
    if (R.determinant() < 0) {
        R = -R;
    }
    Vector3d t = p1 - R * p2;

    // change lidar pose
    Sophus::SE3d T(R, t);
    cout << "initial trans: \n" << T.matrix() << endl;
    for (auto& kfp : keyframes_) {
        kfp.second->lidar_pose_ = T * kfp.second->lidar_pose_;
    }
}

void Optimization::LoadLoopCandidates() 
{
    std::ifstream fin("./data/ch9/loops.txt");
    if (!fin) {
        cout << "cannot load file: ./data/ch9/loops.txt" << endl;
        return;
    }

    auto load_SE3 = [](std::istream& f) -> Sophus::SE3d {
        Sophus::SE3d ret;
        double q[4];
        double t[3];
        f >> t[0] >> t[1] >> t[2] >> q[0] >> q[1] >> q[2] >> q[3];
        return Sophus::SE3d(Quaterniond(q[3], q[0], q[1], q[2]), Vector3d(t[0], t[1], t[2]));
    };

    while (fin.eof() == false) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            break;
        }

        std::stringstream ss;
        ss << line;

        LoopCandidate lc;
        ss >> lc.idx1_ >> lc.idx2_ >> lc.ndt_score_;
        lc.Tij_ = load_SE3(ss);
        loop_candidates_.emplace_back(lc);
    }

    cout << "loaded loops: " << loop_candidates_.size() << endl;
}