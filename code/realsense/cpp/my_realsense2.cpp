#include "../my_realsense2.hpp"


rs2_data *rs2_picData, *rs2_imuData;
void realsenseInit(void)
{
/* 以下是获取图像信息 ******************************************************************************************************/

    //创建图像数据集的内存空间
    rs2_picData = new rs2_data;

    //图像配置
    rs2::config pic_config;
    pic_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pic_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pic_config.disable_stream(RS2_STREAM_ACCEL);
    pic_config.disable_stream(RS2_STREAM_GYRO);   

    //图像管道启动
    rs2_picData->pipelineProfile = rs2_picData->pipeline.start(pic_config);

    //创建帧空间
    rs2_picData->colorFrame_ptr = (rs2::video_frame *)malloc(sizeof(rs2::video_frame));
    rs2_picData->depthFrame_ptr = (rs2::depth_frame *)malloc(sizeof(rs2::depth_frame));

    //获取数据流配置
    rs2_picData->colorStream_profile = rs2_picData->pipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_picData->depthStream_profile = rs2_picData->pipelineProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取标定参数
    rs2_picData->color_intrinsics = rs2_picData->colorStream_profile.get_intrinsics();
    rs2_picData->depth_intrinsics = rs2_picData->depthStream_profile.get_intrinsics();







/* 以下是获取IMU信息 ******************************************************************************************************/

    //配置IMU数据集的内存空间
    rs2_imuData = new rs2_data;

    //IMU配置  
    rs2::config imu_config;
    imu_config.disable_stream(RS2_STREAM_COLOR);
    imu_config.disable_stream(RS2_STREAM_DEPTH);
    imu_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    imu_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);   

    //启动的方法，传入给管道
    auto helpStart = [&](rs2::frame frame)
    {
        auto motion = frame.as<rs2::motion_frame>();

        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            double ts = motion.get_timestamp();
            rs2_imuData->gyro = motion.get_motion_data();
        }
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            rs2_imuData->acc = motion.get_motion_data();
        }
    };

    //启动IMU设备，获得配置
    rs2_imuData->pipeline.start(imu_config, helpStart);
}



/**
 * 更新帧
*/
void updateFrame(void)
{
    //对齐器
    rs2::align align2color(RS2_STREAM_COLOR);

    //获得新的帧集
    rs2_picData->framesets          = rs2_picData->pipeline.wait_for_frames();
    rs2_picData->alignedFrameset    = align2color.process(rs2_picData->framesets);

    // //获得新的帧集
    // drInf.framesets = drInf.pipeline.wait_for_frames();

    // //获取新的数据
    // rs2::video_frame    colorFrame_temp     = drInf.framesets.get_color_frame();
    // rs2::depth_frame    depthFrame_temp     = drInf.framesets.get_depth_frame();
}


rs2_data *getPicData()
{
    rs2_data *result = rs2_picData;
    return result;
}


rs2_data *getIMUData()
{
    rs2_data *result = rs2_imuData;
    return result;
}
