#pragma once
#include "common/basetype.h"
#include "sensor/imu.h"
#include "sync/cloud_convert.h"

/// IMU 数据与雷达同步
struct MeasureGroup 
{
    MeasureGroup() { this->lidar_.reset(new PointCloudType()); };
    double lidar_begin_time_ = 0;   // 雷达包的起始时间
    double lidar_end_time_ = 0;     // 雷达的终止时间
    CloudPtr lidar_ = nullptr;  // 雷达点云
    std::deque<IMUPtr> imu_;        // 上一时时刻到现在的IMU读数
};

/**
 * 将激光数据和IMU数据同步
 */
class MessageSync 
{
public:
    using Callback = std::function<void(const MeasureGroup &)>;
    MessageSync(Callback cb) 
    : 
    callback_(cb),
    conv_(new CloudConvert) 
    {

    }
    /// 初始化
    void Init(const std::string &yaml);
    /// 处理IMU数据
    void ProcessIMU(IMUPtr imu) 
    {
        double timestamp = imu->timestamp_;
        if (timestamp < last_timestamp_imu_) {
            cout << "imu loop back, clear buffer" << endl;
            imu_buffer_.clear();
        }
        last_timestamp_imu_ = timestamp;
        imu_buffer_.emplace_back(imu);
    }
    /**
     * 处理sensor_msgs::PointCloud2点云
     * @param msg
     */
    void ProcessCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) 
    {
        if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            cout << "lidar loop back, clear buffer" << endl;
            lidar_buffer_.clear();
        }
        CloudPtr cloud(new PointCloudType());
        pcl::fromROSMsg(*msg, *cloud);
        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar_ = msg->header.stamp.toSec();
        Sync();
    }
private:
    /// 尝试同步IMU与激光数据，成功时返回true
    bool Sync();
private:
    Callback callback_;                             // 同步数据后的回调函数
    std::shared_ptr<CloudConvert> conv_ = nullptr;  // 点云转换
    std::deque<CloudPtr> lidar_buffer_;         // 雷达数据缓冲
    std::deque<IMUPtr> imu_buffer_;                 // imu数据缓冲
    double last_timestamp_imu_ = -1.0;              // 最近imu时间
    double last_timestamp_lidar_ = 0;               // 最近lidar时间
    std::deque<double> time_buffer_;
    bool lidar_pushed_ = false;
    MeasureGroup measures_;
    double lidar_end_time_ = 0;
};