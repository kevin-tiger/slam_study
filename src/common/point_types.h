#pragma once
#include "common/basetype.h"

// 定义系统中用到的点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// 点云到Eigen的常用的转换函数
inline Vector3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vector3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }