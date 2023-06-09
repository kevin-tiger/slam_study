#pragma once
#include "common/basetype.h"

struct IMU 
{
    IMU() = default;
    IMU(double t, const Vector3d& gyro, const Vector3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vector3d gyro_ = Vector3d::Zero();
    Vector3d acce_ = Vector3d::Zero();
};
using IMUPtr = std::shared_ptr<IMU>;