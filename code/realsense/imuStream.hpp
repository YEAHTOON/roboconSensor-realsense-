#ifndef __IMUSTREAM_H_
#define __IMUSTREAM_H_

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <string.h>
#include <stdlib.h>
#include <iostream>

#include "my_realsense2.hpp"
#include "../dataFrame/dataFrame.hpp"

typedef struct euler
{
    float yaw;
    float roll;
    float pitch;
} euler;

euler getEuler(float roll_x, float pitch_y, float yaw_z);
euler getCameraEuler(void);

#endif