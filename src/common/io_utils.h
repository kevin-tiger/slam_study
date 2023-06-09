#pragma once
#include "common/basetype.h"
#include "sensor/imu.h"
#include "sensor/odom.h"
#include "sensor/gnss.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "common/utm_convert.h"

/// 枚举本书用到的一些数据集
enum class DatasetType {
    UNKNOWN = -1,
    NCLT = 0,   // NCLT: http://robots.engin.umich.edu/nclt/
    KITTI = 1,  // Kitti:
    ULHK = 3,   // https://github.com/weisongwen/UrbanLoco
    UTBM = 4,   // https://epan-utbm.github.io/utbm_robocar_dataset/
    AVIA = 5,   // https://epan-utbm.github.io/utbm_robocar_dataset/
    WXB_3D,     // 3d wxb
};

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
    RosbagIO(const std::string &file_path, DatasetType dataset_type);
    void Go();
    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;
    using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using GNSSHandle = std::function<bool(GNSSPtr)>;
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) 
    {
        process_func_.emplace(topic_name, func);
        return *this;
    }
    RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f)
    {
        string cloud_topic;
        if(dataset_type_ == DatasetType::ULHK)
        {
            cloud_topic = "/velodyne_points_0";
        }
        else if(dataset_type_ == DatasetType::NCLT)
        {
            cloud_topic = "points_raw";
        }
        return AddHandle(cloud_topic, [f](const rosbag::MessageInstance &m) -> bool
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
        string imu_topic;
        if(dataset_type_ == DatasetType::ULHK)
        {
            imu_topic = "/imu/data";
        }
        else if(dataset_type_ == DatasetType::NCLT)
        {
            imu_topic = "imu_raw";
        }
        return AddHandle(imu_topic, [&f, this](const rosbag::MessageInstance &m) -> bool
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
    RosbagIO &AddAutoRTKHandle(GNSSHandle f)
    {
        return AddHandle("gps_rtk_fix", [f, this](const rosbag::MessageInstance &m) -> bool 
        {
            auto msg = m.instantiate<sensor_msgs::NavSatFix>();
            if (msg == nullptr) {
                return false;
            }

            GNSSPtr gnss(new GNSS(msg));
            ConvertGps2UTMOnlyTrans(*gnss);
            if (std::isnan(gnss->lat_lon_alt_[2])) {
                // 貌似有Nan
                return false;
            }

            return f(gnss);
        });
    }
    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }
private:
    std::string bag_file_;
    std::map<std::string, MessageProcessFunction> process_func_;
    DatasetType dataset_type_; 
};