#include "common/basetype.h"
#include "common/io_utils.h"
#include "inc_ndt_lo/incremental_ndt_lo.h"
#include "common/point_cloud_utils.h"
#include "viewer/pangolin_window.h"

string bag_path = "/workspace_18_04/slam_in_autonomous_driving/dataset/sad/ulhk/test3.bag";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    std::shared_ptr<PangolinWindow> ui_ = std::make_shared<PangolinWindow>();
    ui_->Init();
    IncrementalNDTLO::Options options;
    options.ndt3d_options_.nearby_type_ = IncNdt3d::NearbyType::NEARBY6; 
    IncrementalNDTLO ndt_lo(options);
    RosbagIO rosbag_io(bag_path, DatasetType::ULHK);
    cout << "VoxelCloud, voxel_size = 0.1m" << endl;
    rosbag_io.AddAutoPointCloudHandle([&ndt_lo, &ui_](sensor_msgs::PointCloud2::Ptr msg) -> bool 
    {
        cout << fixed << setprecision(3) << "cloud_" << msg->header.stamp.toSec() << endl;
        Sophus::SE3d pose;
        CloudPtr cloud = VoxelCloud(PointCloud2ToCloudPtr(msg));
        ndt_lo.AddCloud(cloud, pose);
        ui_->UpdateScan(cloud, pose);  // 转成Lidar Pose传给UI
        return true;
    });
    rosbag_io.Go();
    cout << "app end" << endl;
    while(1);
    return 0;
}