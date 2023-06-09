#include "common/basetype.h"
#include "common/io_utils.h"

string txt_path = "./data/ch3/10.txt";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    TxtIO io(txt_path);
    io.SetIMUProcessFunc([&](const IMU& imu) 
    {
        // cout << fixed << setprecision(3) << "imu_" << imu.timestamp_ << endl;
    }
    );
    io.SetGNSSProcessFunc([&](const GNSS& gnss)
    {
        // cout << fixed << setprecision(3) << "gnss_" << gnss.unix_time_ << endl;
    }
    );
    io.SetOdomProcessFunc([&](const Odom& odom) 
    {
        // cout << fixed << setprecision(3) << "odom_" << odom.timestamp_ << endl;
    }
    );
    io.Go();
    cout << "app end" << endl;
    while(1);
    return 0;
}