#include "../picStream.hpp"


// /**
//  * 文件结构体
// */
// picStreamData *the_struct;
// picStreamData *picStreamData_Init(void)
// {
//     the_struct = new picStreamData;
//     return the_struct;
// }


/**
 * 使用多线程，可能会等待阻塞，用完图片必须立即释放！
*/
cv::Mat *getPic(void)
{
    //从RS2获得的图像数据
    rs2_data *picDataFromRS2 = getPicData();

    //彩色帧和深度帧
    rs2::video_frame RS2_colorFrame = picDataFromRS2->alignedFrameset.get_color_frame();

    //opencv表示的彩色图
    cv::Mat *result =  new cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)RS2_colorFrame.get_data(), cv::Mat::AUTO_STEP);

    return result;
}



//获取对应坐标的相机坐标系下的坐标（三维）
threeD_point get_CameraBased_XYZ(int x, int y)
{
    threeD_point result;

    //从RS2获得的图像数据
    rs2_data *picDataFromRS2 = getPicData();

    //获取深度图
    rs2::depth_frame RS2_aligned_depthFrame =  picDataFromRS2->alignedFrameset.get_depth_frame();

    //获得深度图参数
    rs2::video_stream_profile depth_stream_profile = RS2_aligned_depthFrame.get_profile().as<rs2::video_stream_profile>();
    const auto depthIntrinsics = depth_stream_profile.get_intrinsics();

    //点的深度
    float depth = RS2_aligned_depthFrame.get_distance(x, y);
    
    //获得三维坐标
    float input[2] = {(float)x, (float)y};
    float output[3] = {0.0f};
    rs2_deproject_pixel_to_point(output, &depthIntrinsics, input, depth);

    result.x = output[0];
    result.y = output[1];
    result.z = output[2];

    return result;
}



/**
 * 将rs2的点集转换为pcl的点云
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr rs2_points__to__pcl(const rs2::points& rs2_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr the_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    //获取深度图参数，设置点云参数
    auto rs2_depthFrame_profile = rs2_points.get_profile().as<rs2::video_stream_profile>();     //获取配置
    the_pc_ptr->width           = rs2_depthFrame_profile.width();
    the_pc_ptr->height          = rs2_depthFrame_profile.height();
    the_pc_ptr->is_dense        = false;
    the_pc_ptr->points.resize(rs2_points.size());

    //将来自rs2的点集数据通过vertices，转变为pcl中的点云
    auto vertices_ptr = rs2_points.get_vertices();
    for (auto& p : the_pc_ptr->points)
    {
        p.x = vertices_ptr->x;
        p.y = vertices_ptr->y;
        p.z = vertices_ptr->z;
        vertices_ptr++;
    }

    return the_pc_ptr;
}


/**
 * 
 * 
 * 获得点云
 * 需要释放!!
 * 
 * 
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(void)
{
    //获取点云
    rs2::pointcloud     rs2_pointCloud;                                                            
    rs2::depth_frame    depthFrame = getPicData()->alignedFrameset.get_depth_frame();
    rs2::points         rs2_points = rs2_pointCloud.calculate(depthFrame);   

    //点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr the_pc_ptr = rs2_points__to__pcl(rs2_points);

    return the_pc_ptr;
}
