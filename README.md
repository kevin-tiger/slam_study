#introduction
this is about mapping and localization by using camera lidar imu wheel_speed sensor. 


function_01 : lidar only odometry by using incremental ndt method.

./bin/run_inc_ndt_lidar
![](./doc/lio_demo.gif)


function_02 : lidar and imu odometry by using loosely coupled eskf method.

./bin/run_eskf_imu_lidar
![](./doc/lio_demo.gif)



# reference
https://github.com/gaoxiang12/slam_in_autonomous_driving