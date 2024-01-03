#include "../imuStream.hpp"

/**
 * 旋转矩阵计算绝对坐标系下的三维向量
*/
dimension3 INS_revolve(dimension3 p, float theta_x, float theta_y, float theta_z)
{
    double cos_x = cos((double)theta_x);
    double sin_x = sin((double)theta_x);
    double cos_y = cos((double)theta_y);
    double sin_y = sin((double)theta_y);
    double cos_z = cos((double)theta_z);
    double sin_z = sin((double)theta_z);

    // 绕X轴反向旋转
    double x1 = p.x;
    double y1 = p.y * cos_x + p.z * sin_x;
    double z1 = -p.y * sin_x + p.z * cos_x;

    // 绕Y轴反向旋转
    double x2 = x1 * cos_y - z1 * sin_y;
    double y2 = y1;
    double z2 = x1 * sin_y + z1 * cos_y;

    // 绕Z轴反向旋转
    double x3 = x2 * cos_z + y2 * sin_z;
    double y3 = -x2 * sin_z + y2 * cos_z;
    double z3 = z2;

    dimension3 result = { x3, y3, z3 };
    return result;
}


/**
 * 使用右手系
*/
euler getEuler(float roll_x, float pitch_y, float yaw_z)
{
    euler result;

    result.yaw = 0.0f;
    float temp = sqrtf(pitch_y*pitch_y + yaw_z*yaw_z);
    if(temp != 0.0f) result.pitch = -atanf(roll_x / temp); 
    if(yaw_z != 0.0f) result.roll = -atanf(pitch_y / yaw_z);

    return result;
}


/**
 * 获得相机姿态角
*/
euler getCameraEuler(void)
{
    euler result;

    //来自IMU的数据
    rs2_data *theData = getIMUData();

    //真正的欧拉角
    result = getEuler(-theData->acc.z, theData->acc.x, theData->acc.y);

    return result;
}

