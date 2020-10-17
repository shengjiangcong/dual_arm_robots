/** ---------------------------------------------------------------------------
 * Copyright (c) 2018~2019, PS-Micro, Co. Ltd.  All rights reserved.
---------------------------------------------------------------------------- */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "object_color_detector/object_detector.h"
#include "object_color_detector/DetectObjectSrv.h"

image_transport::Publisher image_pub_;
probot::vision::ObjectDetector objectDetector_;

//初始化h参数
probot::vision::HSV hsv_object[3];

int ROI_x, ROI_y, ROI_wdith, ROI_height;

// service回调函数，输入参数req，输出参数res
bool detectCallback(object_color_detector::DetectObjectSrv::Request  &req,
                    object_color_detector::DetectObjectSrv::Response &res)
{
    sensor_msgs::ImageConstPtr msg;
    try
    {
        msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", ros::Duration(5.0));
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Timeout waiting for image data: %s", e.what());
        res.result = object_color_detector::DetectObjectSrvResponse::TIMEOUT;
        return false;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    //初始化存储目标检测框的vector
    std::vector<cv::Rect> box;

    // 图像裁剪
    cv::Rect select = cv::Rect(ROI_x, ROI_y, ROI_wdith, ROI_height);
    cv::Mat& img_input = cv_ptr->image;

    img_input = img_input(select);

    //彩色图像的灰度值归一化，颜色空间转换，输出为HSV格式图像
    cv::Mat image2hsv, bgr;
    img_input.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
    cv::cvtColor(bgr, image2hsv,  cv::COLOR_BGR2HSV);

    int box2draw =0;
 
    //红色
    objectDetector_.detection(image2hsv, hsv_object[object_color_detector::DetectObjectSrvRequest::RED_OBJECT], &box);
    //遍历box中所有的Rect，标记检测出来的目标
    if(!box.empty())
    {
        for(; box2draw<box.size(); box2draw++)
        {
             rectangle(img_input, box.at(box2draw), hsv_object[object_color_detector::DetectObjectSrvRequest::RED_OBJECT].color);
             geometry_msgs::Pose targetPose;
             targetPose.position.x = box.at(box2draw).x + cvRound(box.at(box2draw).width/2)  + ROI_x;
             targetPose.position.y = box.at(box2draw).y + cvRound(box.at(box2draw).height/2) + ROI_y;
             res.redObjList.push_back(targetPose);
        }
    }

    //绿色
    objectDetector_.detection(image2hsv, hsv_object[object_color_detector::DetectObjectSrvRequest::GREEN_OBJECT], &box);
    //遍历box中所有的Rect，标记检测出来的目标
    if(!box.empty())
    {
        for(; box2draw<box.size(); box2draw++)
        {
             rectangle(img_input, box.at(box2draw), hsv_object[object_color_detector::DetectObjectSrvRequest::GREEN_OBJECT].color);
             geometry_msgs::Pose targetPose;
             targetPose.position.x = box.at(box2draw).x + cvRound(box.at(box2draw).width/2)  + ROI_x;
             targetPose.position.y = box.at(box2draw).y + cvRound(box.at(box2draw).height/2) + ROI_y;
             res.greenObjList.push_back(targetPose);
        }
    }   

    //蓝色
    objectDetector_.detection(image2hsv, hsv_object[object_color_detector::DetectObjectSrvRequest::BLUE_OBJECT], &box);
    //遍历box中所有的Rect，标记检测出来的目标
    if(!box.empty())
    {
        for(; box2draw<box.size(); box2draw++)
        {
             rectangle(img_input, box.at(box2draw), hsv_object[object_color_detector::DetectObjectSrvRequest::BLUE_OBJECT].color);
             geometry_msgs::Pose targetPose;
             targetPose.position.x = box.at(box2draw).x + cvRound(box.at(box2draw).width/2)  + ROI_x;
             targetPose.position.y = box.at(box2draw).y + cvRound(box.at(box2draw).height/2) + ROI_y;
             res.blueObjList.push_back(targetPose);
        }
    }    

    //cv::imshow("result", img_input);
    //cv::waitKey(100);

    image_pub_.publish(cv_ptr->toImageMsg());

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    nh.param("red/hmin", hsv_object[0].hmin, 0);
    nh.param("red/hmax", hsv_object[0].hmax, 40);

    nh.param("green/hmin", hsv_object[1].hmin, 60);
    nh.param("green/hmax", hsv_object[1].hmax, 140);

    nh.param("blue/hmin", hsv_object[2].hmin, 160);
    nh.param("blue/hmax", hsv_object[2].hmax, 230);

    nh.param("image/ROI_x", ROI_x, 200);
    nh.param("image/ROI_y", ROI_y, 100);
    nh.param("image/ROI_wdith", ROI_wdith, 600);
    nh.param("image/ROI_height", ROI_height, 300);

    hsv_object[0].color = cv::Scalar(0, 0, 255);
    hsv_object[1].color = cv::Scalar(0, 255, 0);
    hsv_object[2].color = cv::Scalar(255, 0, 0);

    //image_transport::Subscriber image_sub = it_.subscribe("/usb_cam/image_raw", 1, &imageCb);
    ros::ServiceServer service = nh.advertiseService("/object_detect", detectCallback);

    image_pub_ = it_.advertise("/object_detection_result", 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
