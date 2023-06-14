#pragma once
#include "common/basetype.h"
#include "sensor/imu.h"
#include "sensor/odom.h"
#include "sensor/gnss.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

class TxtIO 
{
public:
    TxtIO(const std::string &file_path); 
    void Go();
    using IMUProcessFuncType = std::function<void(const IMU &)>;
    using OdomProcessFuncType = std::function<void(const Odom &)>;
    using GNSSProcessFuncType = std::function<void(const GNSS &)>;
    TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) 
    {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }
    TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) 
    {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }
    TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) 
    {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }
private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;
};

class RosbagIO 
{
public:
    RosbagIO(const std::string &file_path);
    void Go();
    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;
    using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) 
    {
        process_func_.emplace(topic_name, func);
        return *this;
    }
    RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f)
    {
        return AddHandle("/velodyne_points_0", [f](const rosbag::MessageInstance &m) -> bool 
        {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg);
        });
    }
    RosbagIO &AddImuHandle(ImuHandle f)
    {
        return AddHandle("/imu/data", [&f, this](const rosbag::MessageInstance &m) -> bool 
        {
            auto msg = m.template instantiate<sensor_msgs::Imu>();
            if (msg == nullptr) {
                return false;
            }
            IMUPtr imu;
            imu = std::make_shared<IMU>(
                msg->header.stamp.toSec(),
                Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
            return f(imu);
        });
    }
private:
    std::string bag_file_;
    std::map<std::string, MessageProcessFunction> process_func_;
};