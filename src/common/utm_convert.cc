#include "utm_convert.h"

bool LatLon2UTM(const Vector2d& latlon, UTMCoordinate& utm_coor) 
{
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);
    // cout << fixed << __LINE__ << " --> " << ret << ", utm_coor.xy_ = " << utm_coor.xy_.transpose() << endl;
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

bool ConvertGps2UTM(GNSS& gps_msg, const Vector2d& antenna_pos, const double& antenna_angle, const Vector3d& map_origin) 
{
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk.z_ = gps_msg.lat_lon_alt_[2];
    /// GPS heading 转成弧度
    double heading = 0;
    if (gps_msg.heading_valid_) {
        heading = (90 - gps_msg.heading_) * math::kDEG2RAD;  // 北东地转到东北天
    }
    /// TWG 转到 TWB
    Sophus::SE3d TBG(Sophus::SO3d::rotZ(antenna_angle * math::kDEG2RAD), Vector3d(antenna_pos[0], antenna_pos[1], 0));
    Sophus::SE3d TGB = TBG.inverse();
    /// 若指明地图原点，则减去地图原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];
    Sophus::SE3d TWG(Sophus::SO3d::rotZ(heading), Vector3d(x, y, z));
    Sophus::SE3d TWB = TWG * TGB;

    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_[0] = TWB.translation().x();
    gps_msg.utm_.xy_[1] = TWB.translation().y();
    gps_msg.utm_.z_ = TWB.translation().z();

    if (gps_msg.heading_valid_) {
        // 组装为带旋转的位姿
        gps_msg.utm_pose_ = TWB;
    } else {
        // 组装为仅有平移的SE3
        // 注意当安装偏移存在时，并不能实际推出车辆位姿
        gps_msg.utm_pose_ = Sophus::SE3d(Sophus::SO3d(), TWB.translation());
    }

    return true;
}