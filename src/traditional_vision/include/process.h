#pragma once 
#include "opencv2/opencv.hpp" // OpenCV头文件
#include "ros/ros.h" // ROS头文件
#include "cv_bridge/cv_bridge.h" // ROS与OpenCV桥接头文件
#include "sensor_msgs/Image.h" // ROS图像消息头文件
#include "geometry_msgs/Point.h" // ROS几何消息头文件
#include <mutex> // 互斥锁头文件
#include <vector> // 向量头文件

#define ARMOUR_PROPORTION_MAX 4.5
#define ARMOUR_PROPORTION_MIN 1.2


// 灯条结构体，存储每个灯条的特征信息
struct Light {
    cv::RotatedRect rects; // 旋转矩形
    cv::Point2f midPoints[2]; // 中线端点
    float slope; // 中线斜率
    float angle; // 旋转角度
    float length; // 灯条长度
    float width; // 灯条宽度
};

cv::Mat image2cv(const sensor_msgs::ImageConstPtr& msg); // 图像转换函数声明
std::vector<std::array<cv::Point2f, 4>> image_processing(const cv::Mat& image); // 图像处理函数声明