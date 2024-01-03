#include "main.hpp"


//信号量
sem_t detectPlane_SEM, detectFeature_SEM;


/**
 * 
 * 更新realsense数据的线程
 * 
 * 
*/
void *realsenseUpdate(void *args)
{
    while(1)
    {
        //更新图像
        updateFrame();

        //释放信号量
        sem_post(&detectFeature_SEM);
        sem_post(&detectPlane_SEM);
    }
}



/**
 * 
 * 
 * 识别直线的线程
 * 
 * 
*/
void *detect_features(void *args)
{
    //计数
    int count = 0;

    while(1)
    {
        //如果图像未更新，在此处堵塞
        sem_wait(&detectFeature_SEM);

        //对原始图像进行二值化
        cv::Mat *source = getPic();     //从realsense获得的原始图像
        cv::cvtColor(*source, *source, cv::COLOR_RGB2GRAY);
        cv::threshold(*source, *source, 150, 255, cv::THRESH_BINARY);

        // 存边缘点集的图片，初始化为黑
        cv::Mat edgePic = source->clone();
        for(int i = 0; i < source->rows; i++){
            for(int j = 0; j < source->cols; j++){
                edgePic.at<uchar>(i,j) = 0;
            }
        }

        /**
         * 前十次程序识别纯黑图，避免bug
        */
        if(count < 10)
        {
            findStraightLine(edgePic, edgePic);
            count++;
        }
        else
        {
            //找到图片中的直线
            findStraightLine(*source, edgePic);
            std::vector<cv::Vec4i> lines;           //直线容器  [0][1]为直线一端点  [2][3]为另一端点
            cv::HoughLinesP(edgePic, lines, 1, CV_PI / 180.0, 120, 100, 50);

            //把直线方程存入set
            Set_of_straightLines theSet;
            theSet.valid_length = 0;
            storeStraightLines(theSet, lines, source->rows);

            //储存最终直线的集合
            Set_of_straightLines finalSet;
            finalSet.valid_length = 0;
            finalSet.data[0] = nullptr;

            //合并theSet中的重复直线，独立直线放入finalSet
            deleteSameLines(theSet, finalSet);

            //检测直线，识别特征
            getFeature(finalSet, source->cols, source->rows);

            //删除直线
            delSet(theSet, lines);

            cv::imshow("source", *source);
            cv::imshow("edgePic", edgePic);
            cv::waitKey(30);
        }

        delete source;
    }
}








/**
 * 
 * 
 * 识别平面的线程
 * 
 * 
*/
void *detect_plane(void *args)
{
    //计数
    int bigcount = 0;

    while(1)
    {
        //如果图像未更新，在此处堵塞
        sem_wait(&detectPlane_SEM);

        //点云指针
        auto the_pc_ptr = getPointCloud();

        //距离滤波
        pcl::PassThrough<pcl::PointXYZ> limitY;
        limitY.setInputCloud(the_pc_ptr);
        limitY.setFilterFieldName("y");
        limitY.setFilterLimits(-0.3, 1.0);
        limitY.filter(*the_pc_ptr);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(the_pc_ptr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.5);
        pass.filter(*the_pc_ptr);

        // 采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_sor;
        voxel_sor.setInputCloud(the_pc_ptr);
        voxel_sor.setLeafSize(0.03f, 0.03f, 0.03f);
        voxel_sor.filter(*the_pc_ptr);

        // 去除离群点
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(the_pc_ptr);
        sor.setMeanK(100);                  // 邻域点数
        sor.setStddevMulThresh(1.0);        // 标准差倍数阈值
        sor.filter(*the_pc_ptr);

        // 创建平面分割对象
        pcl::SACSegmentation<pcl::PointXYZ>     seg;
        pcl::ModelCoefficients::Ptr             coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr                  inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01); 

        //循环中会用到的变量循环外定义
        pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        int source_size = (int)the_pc_ptr->points.size();
        int count = 0;

        float planeDistance[3];
        planeDistance[0] = 0.0f;
        planeDistance[1] = 0.0f;
        planeDistance[2] = 0.0f;

        float plane_k[3][4];

        //循环分割
        int acount = 0;
        while(((double)(int)the_pc_ptr->size() > (double)source_size * 0.1)&&(count < 3))
        {
            acount++;

            seg.setInputCloud(the_pc_ptr);
            seg.segment(*inliers, *coefficients);

            //获得平面
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(the_pc_ptr);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane);

            //获取余点
            extract.setNegative(true);
            extract.filter(*rest_cloud);

            //更新分割输入点云
            *the_pc_ptr = *rest_cloud;

            //有效的平面
            if((int)plane->size() > 20)
            {
                //算平面距离
                float temp = (float)std::sqrt(coefficients->values[0] * coefficients->values[0] +
                                    coefficients->values[1] * coefficients->values[1] +
                                    coefficients->values[2] * coefficients->values[2]);
                if(temp != 0.0) planeDistance[count] = (float)std::abs(coefficients->values[3]) / temp;
                
                //存入平面参数
                for(int j = 0; j < 4; j++)
                {
                    plane_k[count][j] = coefficients->values[j];
                }
            }
            else
            {
                plane.reset();
                continue;
            }

            // pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("3D Viewer1"));
            // viewer_ptr1->addPointCloud<pcl::PointXYZ> (plane, "result_cloud");
            // viewer_ptr1->spin();

            //释放平面
            plane.reset();

            //计数
            count++;
        }

        //释放内存
        rest_cloud.reset();
        coefficients.reset();
        inliers.reset();
        the_pc_ptr.reset();

        //获得姿态角
        euler attitudeAngle = getCameraEuler();

        for(int i = 0; i < 3; i++)
        {
            //获取平面世界坐标系下的法向量
            dimension3 the_vector = {-plane_k[i][2], plane_k[i][0], plane_k[i][1]}; 
            dimension3 world = INS_revolve(the_vector, attitudeAngle.roll, attitudeAngle.pitch, attitudeAngle.yaw);

            if(fabs(world.z) > 0.9f) continue;
            else if((fsqrt(world.y*world.y + world.x*world.x) > 0.9f)&&(planeDistance[i] > 0.1f))
            {
                std::cout << acount++ << std::endl;
                std::cout << "侧平面" << std::endl;
                std::cout << world.x << std::endl;
                std::cout << world.y << std::endl;
                std::cout << world.z << std::endl;
                std::cout << planeDistance[i] << std::endl;
                std::cout << "" << std::endl;
            } 
        }  
    }
}





/**
 * 
 * 
 * 入口
 * 
 * 
 * 
*/
int main(int argc, char **argv)
{
    //信号量初始化
    sem_init(&detectPlane_SEM, 0, 0);
    sem_init(&detectFeature_SEM, 0, 0);

    //realsense初始化
    realsenseInit();

    //创建线程
    pthread_t pid[10];
    pthread_create(pid, NULL, realsenseUpdate, (void *)NULL);       //更新图片
    // pthread_create(pid+1, NULL, detect_features, (void *)NULL);     //识别特征
    pthread_create(pid+2, NULL, detect_plane, (void *)NULL);       //识别平面距离

    int count;
    while(1)
    {
        sleep(1);
    }

    return 0;
}
