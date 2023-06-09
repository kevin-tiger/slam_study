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

message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "Eigen3_VERSION: ${Eigen3_VERSION}")
message(STATUS "Pangolin_VERSION: ${Pangolin_VERSION}")
message(STATUS "PCL_VERSION: ${PCL_VERSION}")

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/viewer/viewer.cc
src/common/io_utils.cc
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(run_eskf_imu_gps app/run_eskf_imu_gps.cc)
target_link_libraries(run_eskf_imu_gps ${PROJECT_NAME})



