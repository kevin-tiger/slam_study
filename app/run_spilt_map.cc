#include "common/basetype.h"
#include "front_end/keyframe.h"
#include "common/point_cloud_utils.h"
#include "common/math_utils.h"

string map_path = "./data/ch9/";
double voxel_size = 0.1;

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    std::map<IdType, KFPtr> keyframes;
    if (!LoadKeyFrames("./data/ch9/keyframes.txt", keyframes)) 
    {
        cout << "failed to load keyframes" << endl;
        return 0;
    }
    std::map<Vector2i, CloudPtr, math::less_vec<2>> map_data;  // 以网格ID为索引的地图数据
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = voxel_size;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    // 逻辑和dump map差不多，但每个点个查找它的网格ID，没有的话会创建
    for (auto& kfp : keyframes) 
    {
        auto kf = kfp.second;
        kf->LoadScan("./data/ch9/");
        CloudPtr cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, kf->opti_pose_2_.matrix());
        // voxel size
        CloudPtr kf_cloud_voxeled(new PointCloudType);
        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*kf_cloud_voxeled);
        cout << "building kf " << kf->id_ << " in " << keyframes.size() << endl;
        // add to grid
        for (const auto& pt : kf_cloud_voxeled->points) 
        {
            int gx = floor((pt.x - 50.0) / 100);
            int gy = floor((pt.y - 50.0) / 100);
            Vector2i key(gx, gy);
            auto iter = map_data.find(key);
            if (iter == map_data.end()) 
            {
                // create point cloud
                CloudPtr cloud(new PointCloudType);
                cloud->points.emplace_back(pt);
                cloud->is_dense = false;
                cloud->height = 1;
                map_data.emplace(key, cloud);
            } 
            else 
            {
                iter->second->points.emplace_back(pt);
            }
        }
    }
    // 存储点云和索引文件
    cout << "saving maps, grids: " << map_data.size() << endl;
    std::system("mkdir -p ./data/ch9/map_data/");
    std::system("rm -rf ./data/ch9/map_data/*");  // 清理一下文件夹
    std::ofstream fout("./data/ch9/map_data/map_index.txt");
    for (auto& dp : map_data) 
    {
        fout << dp.first[0] << " " << dp.first[1] << std::endl;
        dp.second->width = dp.second->size();
        VoxelGrid(dp.second, 0.1);

        // sad::SaveCloudToFile(
        //     "./data/ch9/map_data/" + std::to_string(dp.first[0]) + "_" + std::to_string(dp.first[1]) + ".pcd",
        //     *dp.second);
        pcl::io::savePCDFileASCII("./data/ch9/map_data/" + std::to_string(dp.first[0]) + "_" + std::to_string(dp.first[1]) + ".pcd" + "/map.pcd", *dp.second);
    }
    fout.close();
    cout << "app end" << endl;
    while(1);
    return 0;
}