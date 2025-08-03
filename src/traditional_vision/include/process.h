#pragma once 
#include "opencv2/opencv.hpp" // OpenCV头文件
#include "ros/ros.h" // ROS头文件
#include "cv_bridge/cv_bridge.h" // ROS与OpenCV桥接头文件
#include "sensor_msgs/Image.h" // ROS图像消息头文件
#include "geometry_msgs/Point.h" // ROS几何消息头文件
#include <mutex> // 互斥锁头文件
#include <vector> // 向量头文件

cv::Mat image2cv(const sensor_msgs::ImageConstPtr& msg); // 图像转换函数声明
cv::Mat image_processing(const cv::Mat& image); // 图像处理函数声明