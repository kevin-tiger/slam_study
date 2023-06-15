#include "ndt/ndt_inc.h"

void IncNdt3d::AddCloud(CloudPtr cloud_world) 
{
    std::set<KeyType, math::less_vec<3>> active_voxels;  // 记录哪些voxel被更新
    for (const auto& p : cloud_world->points) 
    {
        auto pt = ToVec3d(p);
        auto key = (pt * options_.inv_voxel_size_).cast<int>();
        auto iter = grids_.find(key);
        if (iter == grids_.end()) 
        {
            // 栅格不存在
            data_.push_front({key, {pt}});
            grids_.insert({key, data_.begin()});
            if (data_.size() >= options_.capacity_) 
            {
                // 删除一个尾部的数据
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } 
        else 
        {
            // 栅格存在，添加点，更新缓存
            iter->second->second.AddPoint(pt);
            data_.splice(data_.begin(), data_, iter->second);  // 更新的那个放到最前
            iter->second = data_.begin();                      // grids时也指向最前
        }
        active_voxels.emplace(key);
    }
    // 更新active_voxels
    // std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
    std::for_each( active_voxels.begin(), active_voxels.end(),
    [this](const auto& key) 
    {
        UpdateVoxel(grids_[key]->second); 
    });
    flag_first_scan_ = false;
}

void IncNdt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

void IncNdt3d::UpdateVoxel(VoxelData& v) 
{
    if (flag_first_scan_) 
    {
        if (v.pts_.size() > 1) 
        {
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vector3d& p) { return p; });
            v.info_ = (v.sigma_ + Matrix3d::Identity() * 1e-3).inverse();  // 避免出nan
        } 
        else 
        {
            v.mu_ = v.pts_[0];
            v.info_ = Matrix3d::Identity() * 1e2;
        }
        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }

    if (v.ndt_estimated_ && v.num_pts_ > options_.max_pts_in_voxel_) 
    {
        return;
    }

    if (!v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) 
    {
        // 新增的voxel
        math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vector3d& p) { return p; });
        v.info_ = (v.sigma_ + Matrix3d::Identity() * 1e-3).inverse();  // 避免出nan
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } 
    else if (v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) 
    {
        // 已经估计，而且还有新来的点
        Vector3d cur_mu, new_mu;
        Matrix3d cur_var, new_var;
        math::ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vector3d& p) { return p; });
        math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);
        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();
        // check info
        Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vector3d lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) {
            lambda[1] = lambda[0] * 1e-3;
        }
        if (lambda[2] < lambda[0] * 1e-3) {
            lambda[2] = lambda[0] * 1e-3;
        }
        Matrix3d inv_lambda = Vector3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

bool IncNdt3d::AlignNdt(Sophus::SE3d& init_pose)
{
    cout << "aligning with inc ndt, pts: " << source_->size() << ", grids: " << grids_.size() << endl;
    assert(grids_.empty() == false);
    Sophus::SE3d pose = init_pose;
    // 对点的索引，预先生成
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) 
    {
        num_residual_per_point = 7;
    }
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) 
    {
        index[i] = i;
    }
    // 我们来写一些并发代码
    int total_size = index.size() * num_residual_per_point;
    for (int iter = 0; iter < options_.max_iteration_; ++iter) 
    {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vector3d> errors(total_size);
        std::vector<Matrix3d> infos(total_size);
        // gauss-newton 迭代
        // 最近邻，可以并发
        // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) 
        std::for_each(index.begin(), index.end(), [&](int idx) 
        {
            auto q = ToVec3d(source_->points[idx]);
            Vector3d qs = pose * q;  // 转换之后的q
            // 计算qs所在的栅格以及它的最近邻栅格
            Vector3i key = (qs * options_.inv_voxel_size_).cast<int>();
            for (int i = 0; i < nearby_grids_.size(); ++i) 
            {
                Vector3i real_key = key + nearby_grids_[i];
                auto it = grids_.find(real_key);
                int real_idx = idx * num_residual_per_point + i;
                /// 这里要检查高斯分布是否已经估计
                if (it != grids_.end() && it->second->second.ndt_estimated_) 
                {
                    auto& v = it->second->second;  // voxel
                    Vector3d e = qs - v.mu_;
                    // check chi2 th
                    double res = e.transpose() * v.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_) 
                    {
                        effect_pts[real_idx] = false;
                        continue;
                    }
                    // build residual
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * Sophus::SO3d::hat(q);
                    J.block<3, 3>(0, 3) = Matrix3d::Identity();
                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } 
                else 
                {
                    effect_pts[real_idx] = false;
                }
            }
        });
        // 累加Hessian和error,计算dx
        double total_res = 0;
        int effective_num = 0;
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Matrix<double, 6, 1> err = Matrix<double, 6, 1>::Zero();
        for (int idx = 0; idx < effect_pts.size(); ++idx) 
        {
            if (!effect_pts[idx]) {
                continue;
            }
            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;
            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }
        if (effective_num < options_.min_effective_pts_) 
        {
            cout << "effective num too small: " << effective_num << endl;
            init_pose = pose;
            return false;
        }
        Matrix<double, 6, 1> dx = H.inverse() * err;
        pose.so3() = pose.so3() * Sophus::SO3d::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();
        if (dx.norm() < options_.eps_) 
        {
            cout << "converged, dx = " << dx.transpose() << endl;
            break;
        }
    }
    init_pose = pose;
    return true;
}

void IncNdt3d::ComputeResidualAndJacobians(const Sophus::SE3d& input_pose, Matrix<double, 18, 18>& HTVH, Matrix<double, 18, 1>& HTVr) 
{
    assert(grids_.empty() == false);
    Sophus::SE3d pose = input_pose;
    // 大部分流程和前面的Align是一样的，只是会把z, H, R三者抛出去而非自己处理
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) 
    {
        num_residual_per_point = 7;
    }
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) 
    {
        index[i] = i;
    }
    int total_size = index.size() * num_residual_per_point;
    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 18>> jacobians(total_size);
    std::vector<Vector3d> errors(total_size);
    std::vector<Matrix3d> infos(total_size);
    // gauss-newton 迭代
    // 最近邻，可以并发
    // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx)
    std::for_each(index.begin(), index.end(), [&](int idx)
    {
        auto q = ToVec3d(source_->points[idx]);
        Vector3d qs = pose * q;  // 转换之后的q
        // 计算qs所在的栅格以及它的最近邻栅格
        Vector3i key = (qs * options_.inv_voxel_size_).cast<int>();
        for (int i = 0; i < nearby_grids_.size(); ++i) 
        {
            Vector3i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            int real_idx = idx * num_residual_per_point + i;
            /// 这里要检查高斯分布是否已经估计
            if (it != grids_.end() && it->second->second.ndt_estimated_) 
            {
                auto& v = it->second->second;  // voxel
                Vector3d e = qs - v.mu_;
                // check chi2 th
                double res = e.transpose() * v.info_ * e;
                if (std::isnan(res) || res > options_.res_outlier_th_) 
                {
                    effect_pts[real_idx] = false;
                    continue;
                }
                // build residual
                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Matrix3d::Identity();                   // 对p
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * Sophus::SO3d::hat(q);  // 对R
                jacobians[real_idx] = J;
                errors[real_idx] = e;
                infos[real_idx] = v.info_;
                effect_pts[real_idx] = true;
            } 
            else 
            {
                effect_pts[real_idx] = false;
            }
        }
    });
    // 累加Hessian和error,计算dx
    double total_res = 0;
    int effective_num = 0;
    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;  // 每个点反馈的info因子
    for (int idx = 0; idx < effect_pts.size(); ++idx) 
    {
        if (!effect_pts[idx]) 
        {
            continue;
        }
        total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effective_num++;
        HTVH += jacobians[idx].transpose() * infos[idx] * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * infos[idx] * errors[idx] * info_ratio;
    }
    cout << "effective: " << effective_num << endl;
}