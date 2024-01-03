cmake_minimum_required(VERSION 3.0)

project(theProgram)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

aux_source_directory(./code/main main)
aux_source_directory(./code/straightLine straightLine)
aux_source_directory(./code/realsense/cpp realsense)

add_executable(theProgram ${main} ${realsense} ${straightLine} ${featureMatch})

target_link_libraries(theProgram ${OpenCV_LIBRARIES})
target_link_libraries(theProgram ${realsense2_LIBRARY})
target_link_libraries(theProgram ${PCL_LIBRARIES})