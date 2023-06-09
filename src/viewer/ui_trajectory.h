#pragma once
#include "common/basetype.h"
#include <pangolin/gl/glvbo.h>

/// UI中的轨迹绘制
class UiTrajectory 
{
   public:
    UiTrajectory(const Vector3f& color) : color_(color) { pos_.reserve(max_size_); }

    /// 增加一个轨迹点
    void AddPt(const Sophus::SE3d& pose);

    /// 渲染此轨迹
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

private:
    int max_size_ = 1e6;           // 记录的最大点数
    std::vector<Vector3f> pos_;       // 轨迹记录数据
    Vector3f color_ = Vector3f::Zero();  // 轨迹颜色显示
    pangolin::GlBuffer vbo_;       // 显存顶点信息
};