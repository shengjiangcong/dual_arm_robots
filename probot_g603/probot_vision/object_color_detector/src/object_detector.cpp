/** ---------------------------------------------------------------------------
 * Copyright (c) 2018~2019, PS-Micro, Co. Ltd.  All rights reserved.
---------------------------------------------------------------------------- */

#include "object_color_detector/object_detector.h"

namespace probot
{
namespace vision
{

ObjectDetector::ObjectDetector()
{
}

ObjectDetector::~ObjectDetector()
{
}

//形态学滤波
void ObjectDetector::homography(cv::Mat image, cv::Mat Opened)
{
    cv::Mat element_9(9, 9, CV_8U, cv::Scalar(1));
    cv::Mat element_3(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(image, Opened, cv::MORPH_OPEN, element_9);
    cv::dilate(Opened, Opened, element_3);
}

void ObjectDetector::detection(cv::Mat image2hsv, HSV hsv_para, std::vector<cv::Rect> *box)
{
    //step3:设置H(色相)阈值，显示Inrange区域对应的输入图像(仅调试使用)
    cv::Mat img_Inrange;
    img_Inrange = cv::Mat::zeros(image2hsv.size(), CV_32FC3);

    cv::Mat mask;
    cv::inRange(image2hsv,
                cv::Scalar(hsv_para.hmin, hsv_para.smin / float(255), hsv_para.vmin / float(255)),
                cv::Scalar(hsv_para.hmax, hsv_para.smax / float(255), hsv_para.vmax / float(255)),
                mask);

    //step4:对mask区域进行形态学-开运算，抑制噪声点
    cv::Mat mask_Opened = cv::Mat::zeros(image2hsv.size(), CV_8U);
    homography(mask, mask_Opened);

    //step5：寻找mask_opened连通区域，标记最大与次大，通过形状因子确定目标区域
    //contours用于保存所有轮廓信息,hierarchy用于记录轮廓之间的继承关系
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_Opened, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    int idx = 0;
    int idx_num = contours.size();  //轮廓总数量
    int idx_left = idx_num;         //筛选后剩余轮廓数量
    char contours_tag[20]={0};      //轮廓对应标签，字符串初始化
    cv::Point origin;               //字符标注原点初始化

    for(int i=0; i<idx_num; i++)
    {
        //计算轮廓对应的的矩形边界
        cv::Rect rect = cv::boundingRect(contours[i]);
        int x = rect.x;
        int y = rect.y;
        int w = rect.width;
        int h = rect.height;
        origin.x = x;
        origin.y = y+15;

        float contour_area = cv::contourArea(contours[i], false );
        float shape_ratio = contour_area*4*CV_PI/contours[i].size()/contours[i].size();

        //轮廓大小(像素数量)、长宽比约束
        if(contours[i].size() < 100 ||w<0.2*h||w>5*h)
        {
            idx_left--;
            continue;
        }
        //轮廓形状因子约束
        else if(shape_ratio < 0.45) //这个值需要人工调整，圆的形状因子为1,越不规整，形状因子越小
        {
            idx_left--;
            continue;
        }
        //如果这个轮廓有父轮廓，说明这个轮廓也不是我们的目标，删除之
        if(hierarchy[i][3] != -1)
        {
            idx_left--;
            continue;
        }
        else
            box->push_back(rect);

    }
    std::cout<<"the total contours:"<<idx_num<<std::endl;
    std::cout<<"the left contours:"<<idx_left<<std::endl;
}

}
}

