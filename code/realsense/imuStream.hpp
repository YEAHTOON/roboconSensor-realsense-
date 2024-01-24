#ifndef __IMUSTREAM_H_
#define __IMUSTREAM_H_

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>

#include "my_realsense2.hpp"
#include "../dataFrame/dataFrame.hpp"

typedef struct euler
{
    float yaw;
    float roll;
    float pitch;
} euler;

/**
 * 互补滤波器
 * 
*/
class complementary_filter
{
public:
    complementary_filter(void)
    {
        this->HighNoise_K = 0.0f;
        this->LowNoise_K = 0.0f;
    }

    complementary_filter(float HighNoise_K, float LowNoise_K)
    {
        this->HighNoise_K = HighNoise_K;
        this->LowNoise_K = LowNoise_K;
    }

    void setK(float HighNoise_K, float LowNoise_K)
    {
        this->HighNoise_K = HighNoise_K;
        this->LowNoise_K = LowNoise_K;
    }

    float filter_realization(float HighNoise_Data, float LowNoise_Data)
    {
        float result = this->HighNoise_K * HighNoise_Data + this->LowNoise_K * LowNoise_Data;
        return result;
    }

private:
    float HighNoise_K, LowNoise_K;
};


class eulerSet
{
public:
    eulerSet(float HighNoise_K, float LowNoise_K)
    {
        this->acc_euler.pitch = 0.0f;
        this->acc_euler.yaw = 0.0f;
        this->acc_euler.roll = 0.0f;

        this->gyro_euler.pitch = 0.0f;
        this->gyro_euler.yaw = 0.0f;
        this->gyro_euler.roll = 0.0f;

        this->accurate_euler.pitch = 0.0f;
        this->accurate_euler.yaw = 0.0f;
        this->accurate_euler.roll = 0.0f;

        this->filter.setK(HighNoise_K, LowNoise_K);
    }

    //计算时间差
    float claculate_deltaT(void)
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        unsigned long long now_time = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        float delta_time = (now_time - last_time) / 1000 ;

        this->last_time = now_time;

        return delta_time;
    }

    void set_accEuler(euler acc_euler)
    {
        this->acc_euler.pitch = acc_euler.pitch;
        this->acc_euler.yaw = acc_euler.yaw;
        this->acc_euler.roll = acc_euler.roll;
    }

    float inArea(float angle)
    {
        if(angle < -1.5708f)
        {
            angle = angle + 3.1415926f;
        } 
        else if(angle > 1.5708f)
        {
            angle = angle - 3.1415926f;
        }

        return angle;

    }

    void calculate_gyroEuler(float gyro_x, float gyro_y, float gyro_z)
    {
        if(cosf(this->accurate_euler.pitch) < 0.0001f) return;

        float yawSpeed = (gyro_y * sinf(this->accurate_euler.roll) + gyro_z * cosf(this->accurate_euler.roll)) / cosf(this->accurate_euler.pitch);   //欧拉角微分方程
        float pitchSpeed = gyro_y * cosf(this->accurate_euler.roll) - gyro_z * sinf(this->accurate_euler.roll);
        float rollSpeed = (gyro_y * sinf(this->accurate_euler.roll) + gyro_z * cosf(this->accurate_euler.roll)) * tanf(this->accurate_euler.pitch) + gyro_x;

        float delta_time = claculate_deltaT();
        this->gyro_euler.pitch = this->accurate_euler.pitch + pitchSpeed * delta_time;
        this->gyro_euler.roll = this->accurate_euler.roll + rollSpeed * delta_time;
        this->gyro_euler.yaw = this->accurate_euler.yaw + yawSpeed * delta_time;

        this->gyro_euler.pitch = this->inArea(this->gyro_euler.pitch);
        this->gyro_euler.roll = this->inArea(this->gyro_euler.roll);
        this->gyro_euler.yaw = this->inArea(this->gyro_euler.yaw);

    }

    void ResetAccurateEuler(void)
    {
        //重置时间
        claculate_deltaT();

        //来自IMU的数据
        rs2_data *theData = getIMUData();
        float acc_x = -theData->acc.z;
        float acc_y = theData->acc.x;
        float acc_z = theData->acc.y;

        //加速度欧拉角
        euler acc_euler;
        acc_euler.yaw = 0.0f;
        float temp = sqrtf(acc_y*acc_y + acc_z*acc_z);
        if(temp != 0.0f) acc_euler.pitch = -atanf(acc_x / temp); 
        if(acc_z != 0.0f) acc_euler.roll = -atanf(acc_y / acc_z);

        this->accurate_euler.pitch = acc_euler.pitch;
        this->accurate_euler.yaw = acc_euler.yaw;
        this->accurate_euler.roll = acc_euler.roll;
    }

    euler get_AccurateEuler(void)
    {
        accurate_euler.yaw = this->filter.filter_realization(gyro_euler.yaw, acc_euler.yaw);
        accurate_euler.pitch = this->filter.filter_realization(gyro_euler.pitch, acc_euler.pitch);
        accurate_euler.roll = this->filter.filter_realization(gyro_euler.roll, acc_euler.roll);

        return this->accurate_euler;
    }

private:
    euler gyro_euler, acc_euler, accurate_euler;
    complementary_filter filter;
    unsigned long long last_time;
};

euler getEuler(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, eulerSet &theEuler);
euler getCameraEuler(eulerSet &theEuler);

#endif