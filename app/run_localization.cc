#include "common/basetype.h"
#include "fusion/fusion.h"
#include "common/io_utils.h"

string config_yaml = "./config/mapping.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;

    Fusion fusion(config_yaml);
    fusion.Init();

    RosbagIO rosbag_io(bag_path_);
    rosbag_io.AddAutoRTKHandle([this](GNSSPtr gnss) 
    {
        return true;
    });


    rosbag_io.Go();

    cout << "app end" << endl;
    while(1);
    return 0;
}