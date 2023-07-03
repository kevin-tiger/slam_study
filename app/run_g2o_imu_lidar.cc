#include "common/basetype.h"
#include "common/io_utils.h"
#include "imu_preinteg/lio_preinteg.h"


string bag_path = "/workspace_18_04/slam_in_autonomous_driving/dataset/sad/ulhk/test3.bag";
string config_file = "./config/velodyne_ulhk.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    LioPreinteg lio;
    lio.Init(config_file);
    RosbagIO rosbag_io(bag_path, DatasetType::ULHK);
    rosbag_io.AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool 
    {
        // cout << fixed << setprecision(3) << "cloud_" << cloud->header.stamp.toSec() << endl;
        lio.PCLCallBack(cloud);
        return true;
    }
    );
    rosbag_io.AddImuHandle([&](IMUPtr imu) 
    {
        // cout << fixed << setprecision(3) << "imu_" << imu->timestamp_ << endl;
        lio.IMUCallBack(imu);
        return true;
    }
    );
    rosbag_io.Go();
    cout << "app end" << endl;
    while(1);
    return 0;
}