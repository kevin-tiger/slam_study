#include "common/basetype.h"
#include "common/io_utils.h"
#include "init/static_imu_init.h"
#include "common/utm_convert.h"
#include "viewer/pangolin_window.h"
#include "imu_preinteg/gins_pre_integ.h"

string txt_path = "./data/ch3/10.txt";
double antenna_angle = 12.06;
double antenna_pox_x = -0.17;
double antenna_pox_y = -0.20;
bool with_odom = true;

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    std::shared_ptr<PangolinWindow> ui = nullptr;
    ui = std::make_shared<PangolinWindow>();
    ui->Init();
    StaticIMUInit imu_init;  
    bool imu_inited = false, gnss_inited = false;
    Vector2d antenna_pos(antenna_pox_x, antenna_pox_y);
    bool first_gnss_set = false;
    Vector3d origin = Vector3d::Zero();
    GinsPreInteg::Options gins_options;
    GinsPreInteg gins(gins_options);
    TxtIO io(txt_path);
    io.SetIMUProcessFunc([&](const IMU& imu) 
    {
        // cout << fixed << setprecision(3) << "imu_" << imu.timestamp_ << endl;
          /// IMU 处理函数
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }
          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置GINS
              GinsPreInteg::Options options;
              options.preinteg_options_.init_bg_ = imu_init.GetInitBg();
              options.preinteg_options_.init_ba_ = imu_init.GetInitBa();
              options.gravity_ = imu_init.GetGravity();
              gins.SetOptions(options);
              imu_inited = true;
              return;
          }
          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }
          /// GNSS 也接收到之后，再开始进行预测
          gins.AddImu(imu);
          auto state = gins.GetState();
          ui->UpdateNavState(state);
          usleep(5e2);
    }
    );
    io.SetGNSSProcessFunc([&](const GNSS& gnss)
    {
        // cout << fixed << setprecision(3) << "gnss_" << gnss.unix_time_ << endl;
            /// GNSS 处理函数
            if (!imu_inited) {
                return;
            }
            GNSS gnss_convert = gnss;
            if (!ConvertGps2UTM(gnss_convert, antenna_pos, antenna_angle) || !gnss_convert.heading_valid_) {
                return;
            }
            /// 去掉原点
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;
            gins.AddGnss(gnss_convert);
            auto state = gins.GetState();
            ui->UpdateNavState(state);
            usleep(1e3);
            gnss_inited = true;
    }
    );
    io.SetOdomProcessFunc([&](const Odom& odom) 
    {
        // cout << fixed << setprecision(3) << "odom_" << odom.timestamp_ << endl;
        imu_init.AddOdom(odom);

        if (imu_inited && gnss_inited) {
            gins.AddOdom(odom);
        }
    }
    );
    io.Go();
    cout << "app end" << endl;
    while(1);
    return 0;
}