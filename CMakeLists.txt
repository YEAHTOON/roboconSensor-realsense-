cmake_minimum_required(VERSION 3.0)

project(theProgram)

SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") # 定义Release编译参数

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

aux_source_directory(./code/main main)
aux_source_directory(./code/straightLine straightLine)
aux_source_directory(./code/realsense/cpp realsense)
aux_source_directory(./code/net net)

add_executable(theProgram ${main} ${realsense} ${straightLine} ${featureMatch} ${net})

target_link_libraries(theProgram ${OpenCV_LIBRARIES})
target_link_libraries(theProgram ${realsense2_LIBRARY})
target_link_libraries(theProgram ${PCL_LIBRARIES})