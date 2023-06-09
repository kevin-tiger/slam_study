#pragma once
#include "common/basetype.h"
#include <pangolin/gl/glvbo.h>

/// 在UI里显示的小车
class UiCar {
   public:
    UiCar(const Vector3f& color) : color_(color) {}

    /// 设置小车 Pose，重设显存中的点
    void SetPose(const Sophus::SE3d& pose);

    /// 渲染小车
    void Render();

   private:
    Vector3f color_;
    pangolin::GlBuffer vbo_;  // buffer data

    static std::vector<Vector3f> car_vertices_;  // 小车的顶点
};
