#include "common/basetype.h"
#include "common/io_utils.h"
#include "init/static_imu_init.h"
#include "eskf/eskf.hpp"
#include "common/utm_convert.h"
#include "viewer/pangolin_window.h"

string txt_path = "./data/ch3/10.txt";
double antenna_angle = 12.06;
double antenna_pox_x = -0.17;
double antenna_pox_y = -0.20;
bool with_odom = true;

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    std::shared_ptr<PangolinWindow> viewer_ptr = nullptr;
    viewer_ptr = std::make_shared<PangolinWindow>();
    viewer_ptr->Init();
    StaticIMUInit imu_init;  
    ESKFD eskf;
    bool imu_inited = false, gnss_inited = false;
    Vector2d antenna_pos(antenna_pox_x, antenna_pox_y);
    bool first_gnss_set = false;
    Vector3d origin = Vector3d::Zero();
    TxtIO io(txt_path);
    io.SetIMUProcessFunc([&](const IMU& imu) 
    {
        // cout << fixed << setprecision(3) << "imu_" << imu.timestamp_ << endl;
        if (!imu_init.InitSuccess()) {
            imu_init.AddIMU(imu);
            return;
        }
        /// 需要IMU初始化
        if (!imu_inited) {
            // 读取初始零偏，设置ESKF
            ESKFD::Options options;
            // 噪声由初始化器估计
            options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
            options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
            eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
            imu_inited = true;
            return;
        }
        if (!gnss_inited) {
            /// 等待有效的RTK数据
            return;
        }
        /// GNSS 也接收到之后，再开始进行预测
        eskf.Predict(imu);
        auto state = eskf.GetNominalState();
        viewer_ptr->UpdateNavState(state);
        usleep(1e3);
    }
    );
    io.SetGNSSProcessFunc([&](const GNSS& gnss)
    {
        // cout << fixed << setprecision(3) << "gnss_" << gnss.unix_time_ << endl;
        if (!imu_inited) {
            return;
        }
        GNSS gnss_convert = gnss;
        if (!ConvertGps2UTM(gnss_convert, antenna_pos, antenna_angle) || !gnss_convert.heading_valid_) 
        {
            return;
        }
        /// 去掉原点
        if (!first_gnss_set) 
        {
            origin = gnss_convert.utm_pose_.translation();
            first_gnss_set = true;
            gnss_inited = true;
            cout << "gnss_inited = true" << endl;
        }
        gnss_convert.utm_pose_.translation() -= origin;
        // 要求RTK heading有效，才能合入ESKF
        eskf.ObserveGps(gnss_convert);
        auto state = eskf.GetNominalState();
        viewer_ptr->UpdateNavState(state);
    }
    );
    io.SetOdomProcessFunc([&](const Odom& odom) 
    {
        // cout << fixed << setprecision(3) << "odom_" << odom.timestamp_ << endl;
        imu_init.AddOdom(odom);
        if (with_odom && imu_inited && gnss_inited) 
        {
            eskf.ObserveWheelSpeed(odom);
            auto state = eskf.GetNominalState();
            viewer_ptr->UpdateNavState(state);
        }
    }
    );
    io.Go();
    cout << "app end" << endl;
    while(1);
    return 0;
}