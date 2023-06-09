#pragma once
#include "common/basetype.h"
#include "sensor/imu.h"
#include "sensor/odom.h"
#include "sensor/gnss.h"

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