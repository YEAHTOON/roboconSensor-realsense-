#ifndef __DEPTHSTREAM_H_
#define __DEPTHSTREAM_H_

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <string.h>
#include <stdlib.h>
#include <iostream>

class rs2_data
{
public:
    rs2::pipeline               pipeline;
    rs2::pipeline_profile       pipelineProfile;
    rs2::video_stream_profile   colorStream_profile;
    rs2::video_stream_profile   depthStream_profile;
    rs2_intrinsics              depth_intrinsics;
    rs2_intrinsics              color_intrinsics;
    rs2::frameset               framesets;
    rs2::frameset               alignedFrameset;
    rs2_vector                  acc;
    rs2_vector                  gyro;
    rs2::video_frame            *colorFrame_ptr;
    rs2::depth_frame            *depthFrame_ptr;
};

void realsenseInit(void);
void updateFrame(void);
rs2_data *getPicData();
rs2_data *getIMUData();

#endif