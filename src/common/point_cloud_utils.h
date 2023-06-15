#pragma once
#include "common/basetype.h"

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud);