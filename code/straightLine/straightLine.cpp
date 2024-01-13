#include "straightLine.hpp"
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

/**
 * 判断一个点是不是边缘点
*/
bool ifEdgePoint(imgPicture *theImage, int x, int y)
{
    if(theImage->data[y+1][x] != lineColor) return true;
    if(theImage->data[y-1][x] != lineColor) return true;
    if(theImage->data[y][x+1] != lineColor) return true;
    if(theImage->data[y][x-1] != lineColor) return true;
    return false;
}

/**
 * 判断一个点是不是要求的颜色
*/
bool ifCatchColor(imgPicture *theImage, int x, int y)
{
    if((*(*(theImage->data + y) + x) == lineColor)&&(x < theImage->cols - 1)&&(x > 1)&&(y < theImage->rows - 1)&&(y > 1))
    { 
        return true;
    }
    return false;
}


/**
 * 查看一个点是不是被访问过了
*/
bool ifVisited(imgPicture *theImage, int x, int y)
{
    if(*(*(theImage->visitedPoints + y) + x) == 1) return true;
    return false;
}


/**
 * 计算点的斜率
*/
float calculateK(imgPicture *theImage, int x, int y, int size)
{

    point highP, lowP;
    highP.x = x;
    highP.y = y;
    lowP.x = x;
    lowP.y = y;

    for(int Xchange = (1-size)/2; Xchange < (size+1)/2; Xchange++)
    {
        for(int Ychange = (1-size)/2; Ychange < (size+1)/2; Ychange++)
        {
            if((x+Xchange > 3)&&(x+Xchange < theImage->cols - 3) && (y+Ychange > 3)&&(y+Ychange < theImage->rows - 3))
            {
                if(ifEdgePoint(theImage, x+Xchange, y+Ychange))
                {
                    if (y+Ychange > highP.y)
                    {
                        highP.x = x+Xchange;
                        highP.y = y+Ychange;
                    }
                    else if(y+Ychange < lowP.y)
                    {
                        lowP.x = x+Xchange;
                        lowP.y = y+Ychange;
                    }
                }
            }
            else
            {
                continue;
            }
        }
    }

    float a = (highP.y - lowP.y);
    float b = (highP.x - lowP.x);
    float result;

    //无效情况
    if((a == 0)&&(b == 0)) return 6.66f;        

    if(b != 0) result = a / b;
    else
    {
        return a * 1000.0;          //k很大
    }
    // std::cout << a << std::endl;
    // std::cout << b << std::endl;
    // std::cout << " " << std::endl;

    return result;
}


/**
 * 广度优先算法，能够搜索到边界点
*/
edgePointsArry *breadthSearch_edgePoints(imgPicture *theImage , int x, int y)
{
    //该点
    pointOnPic thePoint;
    thePoint.x = x;     //列数对应了x
    thePoint.y = y;     //行数对应了y
    theImage->visitedPoints[y][x] = 1;

    //第一个节点
    queueNode<pointOnPic> *firstNode = new queueNode<pointOnPic>(thePoint);

    //队列
    queue<pointOnPic> *theQueue = new queue<pointOnPic>(firstNode, firstNode);

    edgePointsArry *edgePoints = new edgePointsArry;
    int count = 0;

    while(1)
    {
        if(ifEdgePoint(theImage, theQueue->front->data.x, theQueue->front->data.y))
        {
            edgePoints->data[count].x   = theQueue->front->data.x;
            edgePoints->data[count].y   = theQueue->front->data.y;
            count++;
        }

        auto inArea=[&](int x, int y){
            if(  ((x < 4)||(x > theImage->cols - 4)) && ((y < 4)||(y > theImage->rows - 4))  ) return false;
            return true;
        };
        auto pushPoint=[&](int x,int y){
            if((!ifVisited(theImage, x, y)) && ifCatchColor(theImage, x, y) && (inArea(x,y))){
                theQueue->push(new queueNode<pointOnPic>(pointOnPic(x, y)));
                theImage->visitedPoints[y][x] = 1;
                *(*(theImage->visitedPoints + y) + x) = 1;
            }
        };

        pushPoint(theQueue->front->data.x + 1, theQueue->front->data.y);
        pushPoint(theQueue->front->data.x - 1, theQueue->front->data.y);
        pushPoint(theQueue->front->data.x, theQueue->front->data.y + 1);
        pushPoint(theQueue->front->data.x, theQueue->front->data.y - 1);

        theQueue->pop();
        
        if(theQueue->ifEmpty()) break;
    }

    delete theQueue;
    theQueue = nullptr;

    return edgePoints;
}



/**
 * 寻找直线
*/
void findStraightLine(cv::Mat &theMat, cv::Mat &show)
{
    //右上角为正方向的图像
    imgPicture *image = new imgPicture(theMat);

    //遍历每一个点
    for(int i = 3; i < image->rows - 3; i++)
    {
        for(int j = 3; j < image->cols - 3; j++)
        {
            //对每个点进行块搜索
            if((image->data[i][j] == lineColor)&&(image->visitedPoints[i][j] == 0)) 
            {
                //广度优先算法获取边缘点
                edgePointsArry *edgePoints = breadthSearch_edgePoints(image, j, i);   

                //将边缘点信息转换为直线信息
                int count = 0;
                while(edgePoints->data[count].x != 0)
                {
                    // //每个点的斜率
                    // float slopeK = calculateK(image, edgePoints->data[count].x, edgePoints->data[count].y, 5);
                    // float angle = atanf(slopeK);        //与x轴的夹角

                    //显示边缘线
                    if((image->rows - edgePoints->data[count].y < 0)||(edgePoints->data[count].x < 0)||(image->rows - edgePoints->data[count].y > image->rows)||(edgePoints->data[count].x > image->cols)) 
                    {
                        count++;
                        continue;
                    }
                    show.at<uchar>(image->rows - edgePoints->data[count].y, edgePoints->data[count].x) = (uchar)255;

                    count++;
                }

                //删除边缘点集合
                delete edgePoints;
            }

        }
    }

    delete image;
    image = nullptr;
}


/**
 * 
 * 对直线进行处理后存入一个set
 * 
 * rows:直线所在的图像的行数
 * 
*/
void storeStraightLines(Set_of_straightLines &set, std::vector<cv::Vec4i> &lines, int rows)
{
    for (size_t i = 0; i < lines.size(); i++) 
    {
        // cv::line(color, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 1, 8);
        point aboveP, belowP;
        aboveP.x = lines[i][0] * (!(lines[i][1] > lines[i][3])) + lines[i][2] * (lines[i][1] > lines[i][3]);
        aboveP.y = lines[i][1] * (!(lines[i][1] > lines[i][3])) + lines[i][3] * (lines[i][1] > lines[i][3]);
        belowP.x = lines[i][0] * (lines[i][1] > lines[i][3]) + lines[i][2] * (!(lines[i][1] > lines[i][3]));
        belowP.y = lines[i][1] * (lines[i][1] > lines[i][3]) + lines[i][3] * (!(lines[i][1] > lines[i][3]));
        set.data[i] = new straightLine(aboveP.x, rows - aboveP.y, belowP.x, rows - belowP.y);
    }
    set.data[lines.size()] = nullptr;

    //set的有效长度
    set.valid_length = lines.size();
}


//删除重复的曲线
void deleteSameLines(Set_of_straightLines &theSet, Set_of_straightLines &finalSet)
{
    //如果输入两直线相似，返回true
    auto ifStraightLineSame=[&](straightLine *a, straightLine *b){
        if(  fabs(atanf(a->getSlope())-atanf(b->getSlope())) < 0.2  &&  fabs(a->getOffset() - b->getOffset()) < 100.0  )
        {
            return true;
        }
        return false;
    };

    //检测theSet中的每一个元素是否重复
    int unitCount_out = 0, unitCount_inner = 0;
    while(theSet.data[unitCount_out] != nullptr)
    {
        unitCount_inner = 0;

        //逐一比较已经确定的直线，舍弃相似的直线
        while((finalSet.data[unitCount_inner] != nullptr) || (finalSet.valid_length == 0))
        {
            // std::cout << unitCount_inner << std::endl;
            if(finalSet.valid_length == 0)
            {
                finalSet.data[0] = theSet.data[unitCount_out];        //添加新的直线
                finalSet.data[1] = nullptr;
                finalSet.valid_length++;
                unitCount_inner++;
                continue;
            }

            //判断为相同
            if(ifStraightLineSame(theSet.data[unitCount_out], finalSet.data[unitCount_inner]))
            {
                //有更高的点
                if(theSet.data[unitCount_out]->abovePoint[1] > finalSet.data[unitCount_inner]->abovePoint[1])
                {
                    finalSet.data[unitCount_inner]->abovePoint[0] = theSet.data[unitCount_out]->abovePoint[0];
                    finalSet.data[unitCount_inner]->abovePoint[1] = theSet.data[unitCount_out]->abovePoint[1];
                }
                //有更低的点
                if(theSet.data[unitCount_out]->belowPoint[1] < finalSet.data[unitCount_inner]->abovePoint[1])
                {
                    finalSet.data[unitCount_inner]->belowPoint[0] = theSet.data[unitCount_out]->belowPoint[0];
                    finalSet.data[unitCount_inner]->belowPoint[1] = theSet.data[unitCount_out]->belowPoint[1];
                }
                break;
            }
            else        //不同
            {
                if(unitCount_inner == finalSet.valid_length - 1)
                {
                    finalSet.data[finalSet.valid_length++] = theSet.data[unitCount_out];        //添加新的直线
                    finalSet.data[finalSet.valid_length + 1] = nullptr;
                }
            }

            unitCount_inner++;
        }

        unitCount_out++;
    }
}

//删除直线集
void delSet(Set_of_straightLines &theSet, std::vector<cv::Vec4i> &lines)
{
    for(size_t i = 0; i < lines.size(); i++)
    {
        // std::cout << lines.size() << std::endl;
        if(theSet.data[i] != nullptr)
        {
            delete theSet.data[i];
            theSet.data[i] = nullptr;
        }
    }
}

/**
 * 相交类型
*/
featrueType ifCross(straightLine &a,straightLine &b)
{
    //这是两条直线方程的共点
    pointOnPic commonPoint = a.getCommonPoint(b);
    if(a.ifHeightValid(commonPoint.y) && b.ifHeightValid(commonPoint.y))
    {
        auto ifSamePoint = [&](pointOnPic &m, pointOnPic &n)
        {
            float deltaX = fabs((float)(m.x - n.x));
            float deltaY = fabs((float)(m.y - n.y));
            if(deltaX*deltaX + deltaY*deltaY < 100.0f) return 1;
            return 0;
        };

        pointOnPic aa, ab, ba, bb;
        aa.x = a.abovePoint[0];
        aa.y = a.abovePoint[1];
        ab.x = a.belowPoint[0];
        ab.y = a.belowPoint[1];
        ba.x = b.abovePoint[0];
        ba.y = b.abovePoint[1];
        bb.x = b.belowPoint[0];
        bb.y = b.belowPoint[1];

        int sameCount = ifSamePoint(commonPoint, aa) + ifSamePoint(commonPoint, ab) + ifSamePoint(commonPoint, ba) + ifSamePoint(commonPoint, bb);

        if(sameCount == 0) return X_cross;
        else if(sameCount == 1) return T_cross;
        else if(sameCount == 2) return L_cross;
        return neverCross;
    }
    else
    {
        return neverCross;
    }
}



/**
 * 
 * 检测特征，获取坐标
 * 
 * 
*/
void getFeature(Set_of_straightLines &finalSet, int cols, int rows)
{
    if(finalSet.valid_length > 1)
    {
        auto inArea = [&](int x, int y)
        {
            if((x < 10)||(x > cols - 11)) return false;
            if((y < 10)||(y > rows - 11)) return false;
            return true;
        };

        //两直线交点在图像上坐标
        pointOnPic commonPoint = finalSet.data[0]->getCommonPoint(*finalSet.data[1]);
        int commonPointX = commonPoint.x;
        int commonPointY = rows - commonPoint.y;      //已经被转换到原先的y轴

        //交点必须在有效范围内
        if(inArea(commonPointX, commonPointY))
        {
            // //获得深度值
            // float commonPointDepth = RS2_aligned_depthFrame.get_distance(commonPointX, commonPointY);

            // //存储相机坐标系下的交点坐标
            // float commonPointXYZ[3] = {0.0f};                               //相机坐标系下的坐标

            // //获取相机坐标系下坐标
            // float commonPointXY_onPic[2] = {(float)commonPointX, (float)commonPointY};                //在图像上的xy轴坐标
            // rs2_deproject_pixel_to_point(commonPointXYZ, &depthIntrinsics, commonPointXY_onPic, commonPointDepth);
            auto result = get_CameraBased_XYZ(commonPointX, commonPointY);

            std::cout << result.x << std::endl;
            std::cout << result.y << std::endl;
            std::cout << result.z << std::endl;
        }

        //相交特征
        featrueType theFeature = ifCross(*finalSet.data[0], *finalSet.data[1]);

        //特征
        if(theFeature == X_cross) std::cout << "X" << std::endl;
        else if(theFeature == T_cross) std::cout << "T" << std::endl;
        else if(theFeature == L_cross) std::cout << "L" << std::endl;
        else std::cout << "NO" << std::endl;
    }
    else std::cout << "NO" << std::endl;

    std::cout << "*********************************************" << std::endl;
}

