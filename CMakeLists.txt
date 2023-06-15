cmake_minimum_required(VERSION 2.8.3)
project(slam_study)
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "Eigen3_VERSION: ${Eigen3_VERSION}")
message(STATUS "Pangolin_VERSION: ${Pangolin_VERSION}")
message(STATUS "PCL_VERSION: ${PCL_VERSION}")

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
pcl_ros
pcl_conversions
 )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${yaml-cpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/viewer/pangolin_window.cc
src/viewer/pangolin_window_impl.cc
src/viewer/ui_trajectory.cc
src/viewer/ui_car.cc
src/viewer/ui_cloud.cc
src/common/io_utils.cc
src/init/static_imu_init.cc
src/common/utm_convert.cc
thirdparty/utm_convert/utm.cc
thirdparty/utm_convert/tranmerc.cc
src/ieskf/lio_ieskf.cc
src/sync/measure_sync.cc
src/sync/cloud_convert.cc
src/common/point_cloud_utils.cc
src/ndt/ndt_inc.cc
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
yaml-cpp
)

# add_executable(run_eskf_imu_gps_wheel app/run_eskf_imu_gps_wheel.cc)
# target_link_libraries(run_eskf_imu_gps_wheel ${PROJECT_NAME})

add_executable(run_ieskf_imu_lidar app/run_ieskf_imu_lidar.cc)
target_link_libraries(run_ieskf_imu_lidar ${PROJECT_NAME})


