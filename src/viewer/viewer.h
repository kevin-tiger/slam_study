#pragma once
#include "common/basetype.h"
#include <pangolin/pangolin.h>

class Viewer
{
public:
    struct Config
    {
        float view_point_x = 0.0;
        float view_point_y = 0.0;
        float view_point_z = 750.0;
        float view_point_f = 500.0;
        float view_center_x = 0.0;
        float view_center_y = 0.0;
        float view_center_z = 0.0;
        float view_up_x = 0.0;
        float view_up_y = 1.0;
        float view_up_z = 0.0;
    };
    Viewer();
    ~Viewer();
    void UpdateOdoPose(Isometry3d& pos);
private:
    void ViewerLoop();
    void ShowPoses(vector<Isometry3d>& vec_pos);
private:
    Config m_config;
    std::thread* m_viewer_thread;
    std::mutex m_data_mutex;
    vector<Isometry3d> mvec_odo_pose;
};