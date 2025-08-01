#pragma once
// stdard C++ headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> 
#include <stdlib.h> 
#include <pthread.h>
// third-party headers
#include "ros/ros.h"
#include "opencv2/opencv.hpp" // OpenCV头文件
#include "cv_bridge/cv_bridge.h" // ROS与OpenCV桥接头文件
#include "sensor_msgs/Image.h" // ROS图像消息头文件
#include "geometry_msgs/Point.h" // ROS几何消息头文件

void drawFPS(cv::Mat& frame, double fps); // 绘制FPS到图像