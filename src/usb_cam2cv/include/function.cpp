#include "usbCam2image.h"

// 绘制FPS到图像
void drawFPS(cv::Mat& frame, double fps) {
    std::string fpsText = std::to_string(static_cast<int>(fps)) + " FPS";
    cv::putText(frame, fpsText, 
                cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 
                1, 
                cv::Scalar(0, 255, 0), 
                2, 
                cv::LINE_AA);
}