#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <list>
#include <set>
#include <execution>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include <random>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "thirdparty/sophus/se2.hpp"
#include "thirdparty/sophus/se3.hpp"
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace cv;
using namespace Eigen;

using IdType = unsigned long;

// 定义系统中用到的点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;
    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

// POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
//                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
//                                       (float, time, time)(std::uint16_t, ring, ring))
