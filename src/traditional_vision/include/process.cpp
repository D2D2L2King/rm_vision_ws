#include "process.h"

// image图像转cv::Mat函数
cv::Mat image2cv(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr -> image;
}

cv::Mat image_processing(const cv::Mat& image) {
    // 图像处理逻辑
    cv::Mat hsv, mask;
    // 这里处理图像比如灰度化、边缘检测等
    // 蓝色HSV范围
    cv::Scalar lower_blue = cv::Scalar(100, 110, 110);
    cv::Scalar upper_blue = cv::Scalar(120, 255, 255);
    // 红色HSV范围
    cv::Scalar lower_red = cv::Scalar(0, 110, 110);
    cv::Scalar upper_red = cv::Scalar(20, 255, 255);

    // 转换为hsv色彩空间
    // 如果需要使用相关颜色的掩码，可以取消注释相关代码
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV); // 将BGR图像转换为HSV色彩空间
    // cv::inRange(hsv, lower_blue, upper_blue, mask); // 根据蓝色范围创建掩码
    cv::inRange(hsv, lower_red, upper_red, mask); // 根据红色范围创建掩码,返回值是二值图像 





    

    return mask;
}