#ifndef __STRAIGHT_LINE_
#define __STRAIGHT_LINE_

#include <opencv4/opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>
#include "../dataFrame/dataFrame.hpp"
#include "../realsense/picStream.hpp"

#define lineColor 0

/**
 * 点
*/
class pointOnPic
{
public:

    pointOnPic()
    {
        this->x = 0;
        this->y = 0;
    }

    pointOnPic(int x, int y)
    {
        this->x = x;
        this->y = y;
    }

    ~pointOnPic()
    {
        
    }

    int x;
    int y;
};



/**
 * 直线
*/
class straightLine
{
public:
    straightLine(int ax, int ay, int bx, int by)
    {
        this->abovePoint[0] = ax;
        this->abovePoint[1] = ay;
        this->belowPoint[0] = bx;
        this->belowPoint[1] = by;
        this->slope = (float)(this->abovePoint[1] - this->belowPoint[1]) / (this->abovePoint[0] - this->belowPoint[0]);
        this->offset = ((float)abovePoint[1] - this->slope * (float)abovePoint[0]);
    }

    straightLine(int ax, int ay, int bx, int by, float slope)
    {
        this->abovePoint[0] = ax;
        this->abovePoint[1] = ay;
        this->belowPoint[0] = bx;
        this->belowPoint[1] = by;
        this->slope = slope;
        this->offset = ((float)abovePoint[1] - this->slope * (float)abovePoint[0]);
    }

    //求斜率
    float getSlope(void)
    {
        return this->slope;
    }

    //求偏移
    float getOffset(void)
    {
        return this->offset;
    }

    pointOnPic *getAbovePoint(void)
    {
        pointOnPic *result = new pointOnPic;
        result->x = this->abovePoint[0];
        result->y = this->abovePoint[1];
        return result;
    }

    pointOnPic *getBelowPoint(void)
    {
        pointOnPic *result = new pointOnPic;
        result->x = this->belowPoint[0];
        result->y = this->belowPoint[1];
        return result;
    }

    //高度是否有效
    bool ifHeightValid(int height)
    {
        if((height <= this->abovePoint[1] + 10)&&(height >= this->belowPoint[1] - 10)) return true;
        return false;
    } 

    /**
     * 求两直线相交点的坐标
     * 如果此点坐标距离某直线的顶点很近，则判断这两点重合
    */
    pointOnPic getCommonPoint(straightLine &anotherLine)
    {
        pointOnPic result;
        result.x = (int) ((this->offset - anotherLine.offset) / (anotherLine.slope - this->slope));
        result.y = (int) ((this->slope * anotherLine.offset - this->offset * anotherLine.slope) / (this->slope - anotherLine.slope));
        return result;
    }

    int abovePoint[2];      //定义直线的上方点
    int belowPoint[2];      //定义直线的下方点
    float slope;
    float offset;
};



typedef struct Set_of_straightLines
{
    straightLine *data[1000];
    int valid_length;
} Set_of_straightLines;


/**
 * 行上下翻转
*/
class imgPicture
{
public:

    /**
     * 输入一个mat，并将其纵坐标翻转，从而使图像右上角为正方向
    */
    imgPicture(cv::Mat &thePic)
    {
        this->rows = thePic.rows;
        this->cols = thePic.cols;

        //申请图像空间
        this->data = (int **)malloc(sizeof(int *) * this->rows);      //申请行空间
        for(int i = 0; i < this->rows; i++)
        {
            *(this->data + i) = (int *)malloc(sizeof(int) * this->cols);      //申请列空间
            for(int j = 0; j < this->cols; j++)
            {
                this->data[i][j] = thePic.at<uchar>(this->rows - i, j);   //两级反转
            }
        }

        //申请访问标志符空间
        this->visitedPoints = (int **)malloc(sizeof(int *) * this->rows);
        for(int i = 0; i < this->rows; i++)
        {
            *(this->visitedPoints + i) = (int *)malloc(sizeof(int) * this->cols);      //申请列空间
            for(int j = 0; j < this->cols; j++)
            {
                this->visitedPoints[i][j] = 0;
            }
        }
    }

    //析构函数
    ~imgPicture(void)
    {
        for(int i = 0; i < this->rows; i++)
        {
            free(*(this->data + i));
        }
        free(this->data);

        for(int i = 0; i < this->rows; i++)
        {
            free(*(this->visitedPoints + i));
        }
        free(this->visitedPoints);
    }

    int **data;
    int **visitedPoints;
    int rows;
    int cols;
};

typedef enum
{
    X_cross,
    T_cross,
    L_cross,
    neverCross,
} featrueType;

edgePointsArry *breadthSearch_edgePoints(imgPicture *theImage , int x, int y);
void findStraightLine(cv::Mat &theMat, cv::Mat &show);
void storeStraightLines(Set_of_straightLines &set, std::vector<cv::Vec4i> &lines, int rows);
void deleteSameLines(Set_of_straightLines &theSet, Set_of_straightLines &finalSet);
void delSet(Set_of_straightLines &theSet, std::vector<cv::Vec4i> &lines);
featrueType ifCross(straightLine &a, straightLine &b);
void getFeature(Set_of_straightLines &finalSet, int cols, int rows);

#endif