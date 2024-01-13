#ifndef __PICSTREAM_H_
#define __PICSTREAM_H_

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <string.h>
#include <stdlib.h>
#include <iostream>

#include "my_realsense2.hpp"
#include "../dataFrame/dataFrame.hpp"

cv::Mat *getPic(void);
threeD_point get_CameraBased_XYZ(int x, int y);

class pictureData
{
    cv::Mat *colorPic;
    cv::Mat *depthPic;
    rs2_intrinsics depthIntrinsics;
};

dimension3 INS_revolve(dimension3 p, float theta_x, float theta_y, float theta_z);
pcl::PointCloud<pcl::PointXYZ>::Ptr rs2_points__to__pcl(const rs2::points& rs2_points);
pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(void);

#endif
