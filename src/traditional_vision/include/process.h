#ifndef PROCESS_H
#define PROCESS_H
#include "opencv2/opencv.hpp" // OpenCV头文件
#include "ros/ros.h" // ROS头文件
#include "cv_bridge/cv_bridge.h" // ROS与OpenCV桥接头文件
#include "sensor_msgs/Image.h" // ROS图像消息头文件
#include "geometry_msgs/Point.h" // ROS几何消息头文件
#include <mutex> // 互斥锁头文件
#include <vector> // 向量头文件
#include <array> // 数组头文件
#include <algorithm> // 算法头文件

#define ARMOUR_PROPORTION_MAX 4.5 // 装甲板灯条匹配长宽比
#define ARMOUR_PROPORTION_MIN 1.2 // 装甲板灯条匹配长宽比

#define TRANSFORM_WIDTH 80// 40 // 几何变换后裁减的图像宽
#define TRANSFORM_HEIGHT 56// 28 // 几何变换后裁减的图像高

#define SMALL_ARMOUR_WIDTH 13.1 // 小装甲板灯条距离
#define SMALL_ARMOUR_HEIGHT 5.5 // 小装甲板灯条距离


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
std::vector<std::array<cv::Point2f, 4>> image_processing(const cv::Mat& image, bool color); // 图像处理函数声明
cv::Mat armour_transform(std::array<cv::Point2f, 4> &array_rect, cv::Mat &image_raw); // 装甲板仿射变换函数声明
void sortPointsClockwise(std::array<cv::Point2f, 4>& array_rect); // 顺时针排序函数声明
cv::Mat TFget(std::array<cv::Point2f, 4>& array_rect, bool select_armour); // 获取装甲板到摄像头4x4的变换矩阵

#endif // PROCESS_H