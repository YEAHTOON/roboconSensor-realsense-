#include "main.hpp"

//子进程的数量
int *children_count;

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
#define x_distance 0.24f
#define y_distance 0.153488f

void *detect_plane(void *args)
{
    //欧拉角的方法与数据
    eulerSet theEuler(0.9f, 0.1f);

    //低通滤波器
    LowPass_Filter lowpass_filter;
    LowPass_Filter lowpass_filter_angle;

    //消息队列
    int msqid = msgget(111, IPC_CREAT|400);

    //计数
    int bigcount = 0;

    while(1)
    {
        

        //通知脚本无问题
        std::cout << "PL normal" << std::endl;

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
        while(((double)(int)the_pc_ptr->size() > (double)source_size * 0.1)&&(count < 3))
        {
            //计数
            count++;

            //点太少就结束
            if(the_pc_ptr->size() > 10)
            {
                seg.setInputCloud(the_pc_ptr);
                seg.segment(*inliers, *coefficients);
            }
            else
            {
                continue;
            }

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
        }

        //释放内存
        rest_cloud.reset();
        coefficients.reset();
        inliers.reset();
        the_pc_ptr.reset();

        //获得姿态角
        if(bigcount == 0)
        {
            theEuler.ResetAccurateEuler();
        }
        euler attitudeAngle = getCameraEuler(theEuler);

        for(int i = 0; i < 3; i++)
        {
            //获取平面世界坐标系下的法向量
            dimension3 the_vector = {-plane_k[i][2], plane_k[i][0], plane_k[i][1]}; 
            dimension3 world = INS_revolve(the_vector, attitudeAngle.roll, attitudeAngle.pitch, 0.0f);

            if(fabs(world.z) > 0.9f) continue;
            else if(((float)sqrt((double)(world.y*world.y + world.x*world.x)) > 0.9f)&&(planeDistance[i] > 0.1f))
            {
                struct Temp
                {
                    long mtype;
                    float mtext[2];
                } temp;

                //角度低通滤波
                float angle_rad = (world.x == 0.0f) ? (3.1415926f / 2.0f):(atanf(world.y / world.x));
                float angle = 180.0f * angle_rad / 3.1415926f;
                if(bigcount == 0)
                {
                    lowpass_filter_angle.set(angle);
                }
                else
                {
                    lowpass_filter_angle.insert(angle);
                }

                //初始低通滤波设置为当前值
                float robot_planeDistance = planeDistance[i] + x_distance * sinf(-angle_rad) + y_distance * cosf(-angle_rad);
                if(bigcount == 0)
                {
                    lowpass_filter.set(robot_planeDistance);
                }
                else    //否则插入新的数据一起滤波
                {
                    lowpass_filter.insert(robot_planeDistance);
                }

                temp.mtype = 2;
                temp.mtext[0] = lowpass_filter.getData();
                temp.mtext[1] = lowpass_filter_angle.getData();

                for(int i = 0; i < *children_count; i++)
                {
                    msgsnd(msqid, &temp, sizeof(temp.mtext), IPC_NOWAIT);
                }
            } 
        }  

        //大循环计数
        bigcount++;
    }
}







/**
 * 处理来自客户端的请求（在主函数中）
*/
void Respond_ClientRequst_MainProgram(uint16_t &mainStatus, dataFromTCP &DataFromTCP, int msqid, pthread_t *pid)
{
    //接收数据缓冲区
    char buff[24] = {0};

    //响应客户端发来的数据
    for(int i = 0; i < DataFromTCP.childcount; i++)
    {
        if(read(DataFromTCP.pipeline_read[i][0], buff, sizeof(buff)) > 0)
        {
            std::cout << "client " << i << " : " << buff << std::endl;

            //改变全局状态
            if(strstr(buff, "create SL"))
            {
                //线程未开启的状态下开启线程
                if((mainStatus & 0x01) == 0x00)
                {
                    pthread_create(pid+1, NULL, detect_features, (void *)NULL);     //识别特征
                    mainStatus |= 0x01;
                    std::cout << "ok" << std::endl;
                }
            }
            else if(strstr(buff, "create PL"))
            {
                //线程未开启的状态下开启线程
                if((mainStatus & 0x02) == 0x00)
                {
                    pthread_create(pid+2, NULL, detect_plane, (void *)NULL);        //识别平面距离
                    mainStatus |= 0x02;
                }
            }
            else if(strstr(buff, "shutdown SL"))
            {
                //线程未开启的状态下关闭线程
                if((mainStatus & 0x01) == 0x01)
                {
                    pthread_cancel(pid[1]);
                    mainStatus &= 0xFE;
                }
            }
            else if(strstr(buff, "shutdown PL"))
            {
                //线程未开启的状态下开启线程
                if((mainStatus & 0x02) == 0x02)
                {
                    pthread_cancel(pid[2]);
                    mainStatus &= 0xFD;
                }
            }

            //改变状态后向子进程通知
            struct Temp
            {
                long mtype;
                uint16_t mtext;
            } temp;
            temp.mtype = mainStatusMsg;
            temp.mtext = mainStatus;
            for(int i = 0; i < *children_count; i++)
            {
                msgsnd(msqid, &temp, sizeof(temp.mtext), 0);
            }
        }
    }
}







/**
 * 处理键盘输入的线程
 * 用于退出进程以防止出现僵尸进程或孤儿进程一直运行
*/
void Process_keyBoard_Input(void *args)
{
    
}



void Check_Command(char *)
{
    
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
/* 启动参数处理-------------------------------------------------------------------------------- */
    // if(argc < 2)
    // {
    //     perror("WHU_ROBONCON: program for realsense arguments too few");
    //     exit(-1);
    // }

//线程、进程间通信-------------------------------------------------------------------------------------
    //防僵尸进程
    signal(SIGCHLD, SIG_IGN);

    //信号量初始化
    sem_init(&detectPlane_SEM, 0, 0);
    sem_init(&detectFeature_SEM, 0, 0);

    //创建消息队列
    int msqid = msgget(12345, IPC_CREAT|400);

//realsense初始化----------------------------------------------------------------------------------------
    realsenseInit();

//线程调度------------------------------------------------------------------------------------------
    //创建TCP线程
    pthread_t pid[10];
    dataFromTCP DataFromTCP;
    pthread_create(pid, NULL, realsenseUpdate, (void *)NULL);               //更新图片
    pthread_create(pid+3, NULL, processTCP, (void *)&DataFromTCP);          //tcp通信

    //子进程的数目
    DataFromTCP.childcount = 0;
    children_count = &(DataFromTCP.childcount);

    //初始默认开启识别平面线程
    uint16_t mainStatus = 0x00;
    pthread_create(pid+2, NULL, detect_plane, (void *)NULL);        //识别平面距离
    mainStatus |= 0x02;

    //主线程，用于响应
    while(1)
    {
        //提示脚本正常运行
        if((mainStatus & 0x02) != 0x02) std::cout << "PL normal" << std::endl;

        //接收请求，改变线程状态
        Respond_ClientRequst_MainProgram(mainStatus, DataFromTCP, msqid, pid);

        //延迟
        usleep(10000);
    }

    return 0;
}
