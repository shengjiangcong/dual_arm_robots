/** ---------------------------------------------------------------------------
 * Copyright (c) 2018~2019, PS-Micro, Co. Ltd.  All rights reserved.
---------------------------------------------------------------------------- */

#ifndef PROBOT_VISION_FRUIT_DETECTOR
#define PROBOT_VISION_FRUIT_DETECTOR


#include<iostream>
#include<string>
#include<stdio.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>


namespace probot
{
namespace vision
{

struct HSV
{
    //色相、饱和度、亮度
    int hmin = 35;
    int hmax = 60;

    int smin = 90;
    int smax = 255;

    int vmin = 60;
    int vmax = 255;

    cv::Scalar color = cv::Scalar(255, 255, 255);
};

class ObjectDetector
{
public:
    ObjectDetector();
    ~ObjectDetector();

    void detection(cv::Mat image2hsv, HSV hsv_para, std::vector<cv::Rect> *box);

private:
    void homography(cv::Mat image, cv::Mat Opened);

};

}
}

#endif
