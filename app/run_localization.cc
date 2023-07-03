#include "common/basetype.h"
#include "fusion/fusion.h"
#include "common/io_utils.h"

string config_yaml = "./config/mapping.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;

    Fusion fusion(config_yaml);
    fusion.Init();

    auto yaml = YAML::LoadFile(config_yaml);
    auto bag_path_ = yaml["bag_path"].as<std::string>();
    RosbagIO rosbag_io(bag_path_);
    rosbag_io.AddAutoRTKHandle([&fusion](GNSSPtr gnss) 
    {
            fusion.ProcessRTK(gnss);
            return true;
    });
    rosbag_io.AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool 
    {
            fusion.ProcessPointCloud(cloud);
            return true;
    });
    rosbag_io.AddImuHandle([&](IMUPtr imu) 
    {
            fusion.ProcessIMU(imu);
            return true;
    });
    rosbag_io.Go();

    cout << "app end" << endl;
    while(1);
    return 0;
}