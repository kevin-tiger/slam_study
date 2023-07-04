#include "common/basetype.h"
#include "front_end/keyframe.h"
#include "common/point_cloud_utils.h"

double voxel_size = 1;  // 0.1
string pose_source = "opti2";
string dump_to = "./data/ch9/";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    std::map<IdType, KFPtr> keyframes;
    if (!LoadKeyFrames("./data/ch9/keyframes.txt", keyframes)) {
        cout << "failed to load keyframes.txt" << endl;
        return -1;
    }
    if (keyframes.empty()) {
        cout << "keyframes are empty" << endl;
        return 0;
    }
    // dump kf cloud and merge
    cout << "merging" << endl;
    CloudPtr global_cloud(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = voxel_size;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    int cnt = 0;
    for (auto& kfp : keyframes) {
        auto kf = kfp.second;
        Sophus::SE3d pose;
        if (pose_source == "rtk") {
            pose = kf->rtk_pose_;
        } else if (pose_source == "lidar") {
            pose = kf->lidar_pose_;
        } else if (pose_source == "opti1") {
            pose = kf->opti_pose_1_;
        } else if (pose_source == "opti2") {
            pose = kf->opti_pose_2_;
        }
        kf->LoadScan("./data/ch9/");
        CloudPtr cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix());
        // voxel size
        CloudPtr kf_cloud_voxeled(new PointCloudType);
        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*kf_cloud_voxeled);
        *global_cloud += *kf_cloud_voxeled;
        kf->cloud_ = nullptr;
        cout << "merging " << cnt << " in " << keyframes.size() << ", pts: " << kf_cloud_voxeled->size()
                  << " global pts: " << global_cloud->size() << endl;
        cnt++;
    }
    if (!global_cloud->empty()) 
    {
        pcl::io::savePCDFileASCII(dump_to + "/map.pcd", *global_cloud);
    }
    cout << "app end" << endl;
    while(1);
    return 0;
}