#include "io_utils.h"

TxtIO::TxtIO(const std::string &file_path)
:
fin(file_path) 
{

}

void TxtIO::Go() 
{
    if (!fin) 
    {
        cout << "未能找到文件" << endl;
        return;
    }
    while (!fin.eof()) 
    {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            // 以#开头的是注释
            continue;
        }
        // load data from line
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU" && imu_proc_) 
        {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            imu_proc_(IMU(time, Vector3d(gx, gy, gz), Vector3d(ax, ay, az)));
        } 
        else if (data_type == "ODOM" && odom_proc_) 
        {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            odom_proc_(Odom(time, wl, wr));
        } 
        else if (data_type == "GNSS" && gnss_proc_) 
        {
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            gnss_proc_(GNSS(time, 4, Vector3d(lat, lon, alt), heading, heading_valid));
        }
    }
}

RosbagIO::RosbagIO(const std::string &file_path)
:
bag_file_(file_path) 
{

}

void RosbagIO::Go() 
{
    rosbag::Bag bag(bag_file_);
    auto view = rosbag::View(bag);
    for (const rosbag::MessageInstance &m : view) 
    {
        // cout << "m.getTopic() = " << m.getTopic() << endl;
        auto iter = process_func_.find(m.getTopic());
        if (iter != process_func_.end()) 
        {
            // cout << "m.getTopic() = " << m.getTopic() << endl;
            iter->second(m);
        }
    }
}
