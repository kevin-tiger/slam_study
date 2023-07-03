# introduction  
this is about mapping and localization by using lidar, imu, gps and wheel speed sensor; by using eskf, ieskf, g2o method.

function_01 : lidar only odometry by using incremental ndt method.   
./bin/run_inc_ndt_lidar  
![](./doc/inc_ndt_lidar.png)

function_02 : lidar and imu odometry by using loosely coupled eskf method.   
./bin/run_eskf_imu_lidar  
![](./doc/eskf_imu_lidar.png)

function_03 : imu, gps and wheel_speed odometry by using loosely coupled eskf method.
./bin/run_eskf_imu_gps_wheel
![](./doc/eskf_imu_gps_wheel.png)

function_04 : imu, gps and wheel_speed odometry by using loosely coupled g2o method.   
./bin/run_g2o_imu_gps_wheel   
![](./doc/g2o_imu_gps_wheel.png)

function_05 : lidar and imu odometry by using tightly coupled ieskf method.   
./bin/run_ieskf_imu_lidar  
![](./doc/ieskf_imu_lidar.png)

<!-- function_06 : lidar and imu odometry by using tightly coupled g2o method.
./run_g2o_imu_lidar -->
<!-- ![](./doc/g2o_imu_lidar.png) -->

function_06 : offline build map   
./bin/run_mapping  

function_07 : online localization based offline map   
./bin/run_localization  

# dataset
https://pan.baidu.com/s/1ky_TDLMvdqJjwHuuuWquFA?pwd=1234    
note: split compress command   
tar -czv test3.bag | split -b 1000m - test3.tar.gz   
cat test3* > test3.tar.gz    
tar -xvf test3.tar.gz    

# docker environment
step_1: installl docker by command -- sudo apt  install docker.io  
step_2: download lidar_slam.tar from path https://pan.baidu.com/s/1ky_TDLMvdqJjwHuuuWquFA?pwd=1234
step_3: import docker image by command -- docker load -i lidar_slam.tar  
step_4: docker run -it -v /home/hongfeng/workspace_18_04/:/workspace_18_04/ -v /tmp/.x11-unix:/tmp/.x11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE --net=host --name lidar_slam lidar_slam:V1.0 /bin/bash  
others useful docker command:  
docker exec -it lidar_slam bash
docker ps -a  
docker start lidar_slam   
docker stop lidar_slam  
docker rm lidar_slam  
docker rmi 8ae5b6462b6d  
save container to image :   
docker commit <container id> <image name>:<tag> -- docker commit ff3e1af39fd3 lidar_slam:V1.0  
docker save -o <filename>.tar <image name>:<tag> -- docker save -o lidar_slam.tar lidar_slam:V1.0  

# hardware
ThinkPad T14 Core i7 8 core, 32G memory

# reference
https://github.com/gaoxiang12/slam_in_autonomous_driving