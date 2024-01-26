#ifndef __MAIN_H_
#define __MAIN_H_

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include <exception>

#include "../realsense/imuStream.hpp"
#include "../realsense/picStream.hpp"
#include "../filter/filter.hpp"
#include "../realsense/my_realsense2.hpp"
#include "../dataFrame/dataFrame.hpp"
#include "../straightLine/straightLine.hpp"
#include "../net/net.hpp"

#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>

#include <pcl/search/kdtree.h>

#include <pcl/surface/concave_hull.h>

#include <pcl/filters/passthrough.h>

#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

#include <sys/prctl.h>
#include <signal.h>

#include <sys/types.h>
#include <wait.h>

#include <mutex>

//给线程间通信用的结构体
typedef struct threadsMedium
{
    int intType[10];
    float floatType[10];
} threadsMedium;

typedef struct status
{
    int Pldetect_on;
    int SLdetect_on;
} status;


#endif
