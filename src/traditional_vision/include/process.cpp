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

std::vector<std::array<cv::Point2f, 4>> image_processing(const cv::Mat& image) {
    // 结果储存动态数组
    std::vector<std::array<cv::Point2f, 4>> result_rect;
    // 图像处理逻辑
    cv::Mat hsv, mask, frame;
    frame = image.clone();
    // 这里处理图像比如灰度化、边缘检测等
    // 蓝色HSV范围
    cv::Scalar lower_blue = cv::Scalar(100, 100, 100); // H:色相, S:饱和度, V:亮度
    cv::Scalar upper_blue = cv::Scalar(125, 255, 255);
    // 红色HSV范围
    cv::Scalar lower_red = cv::Scalar(0, 100, 100);
    cv::Scalar upper_red = cv::Scalar(25, 255, 255);

    // 转换为hsv色彩空间
    // 如果需要使用相关颜色的掩码，可以取消注释相关代码
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV); // 将BGR图像转换为HSV色彩空间
    cv::inRange(hsv, lower_blue, upper_blue, mask); // 根据蓝色范围创建掩码
    // cv::inRange(hsv, lower_red, upper_red, mask); // 根据红色范围创建掩码,返回值是二值图像
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 4); // debug
    // cv::imshow("1", frame); // debug
    std::vector<Light> lights; // 存储所有检测到的灯条

    // 处理每个轮廓
    for (size_t i = 0; i < contours.size(); i++) 
    {
        // 过滤小面积区域
        if (cv::contourArea(contours[i]) < 20) 
            continue;
        // cv::drawContours(frame, contours, i, cv::Scalar(0, 255, 0), 4); // debug

        // 这里对单个灯条进行滤波
        // 计算最小旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);
        Light light; // 创建灯条对象
        light.rects = rect; // 存储旋转矩形

        // 存储角度和尺寸
        light.angle = rect.angle;
        light.length = cv::max(rect.size.width, rect.size.height);
        light.width = cv::min(rect.size.width, rect.size.height);

        // 过滤比例不合适的方框 
        if ((light.length / light.width) < 1.85f || (light.length / light.width) > 15.0f)
            continue;

        // 获取矩形顶点并计算中线
        cv::Point2f vertex[4];
        rect.points(vertex);
        if (rect.size.width > rect.size.height) 
        {
            light.midPoints[0] = (vertex[0] + vertex[1]) * 0.5f;
            light.midPoints[1] = (vertex[2] + vertex[3]) * 0.5f;
        } 
        else 
        {
            light.midPoints[0] = (vertex[1] + vertex[2]) * 0.5f;
            light.midPoints[1] = (vertex[3] + vertex[0]) * 0.5f;
        }

        lights.push_back(light); // 压入动态数组
        // 绘制旋转矩形和中线
        // for (int j = 0; j < 4; j++) 
        // {
        //     cv::line(frame, vertex[j], vertex[(j+1)%4], cv::Scalar(0, 255, 0), 2);
        // }
        cv::line(frame, light.midPoints[0], light.midPoints[1], cv::Scalar(0, 0, 255), 6);
        // cv::circle(frame, light.midPoints[0], 5, cv::Scalar(255, 0, 0), -1);
        // cv::circle(frame, light.midPoints[1], 5, cv::Scalar(255, 0, 0), -1);
    }

        int n = 1; // 步长
        // 两两匹配灯条
        for (size_t i = 0; i < lights.size(); i = i + n) 
        {
            n = 1; // 每次都初始化为1
            for (size_t j = (i + 1); j < lights.size(); j++) 
            {
                Light &light1 = lights[i]; // 引用，避免复制
                Light &light2 = lights[j]; // 引用，避免复制
                // 检查灯条长度差异
                float armour_length = ((norm(light1.midPoints[0] - light2.midPoints[0])) > (norm(light1.midPoints[0] - light2.midPoints[1]))) ? (norm(light1.midPoints[0] - light2.midPoints[1])) : (norm(light1.midPoints[0] - light2.midPoints[0])); // 算出灯条梯形的宽
                // 这里对灯条匹配条件进行筛选
                float light1_distance = norm(light1.midPoints[0] - light1.midPoints[1]); // 灯条长度
                float light2_distance = norm(light2.midPoints[0] - light2.midPoints[1]); // 灯条长度
                if (light1_distance < 10) // 单个灯条的长度限制
                    continue; 
                if (abs(light1_distance - light2_distance) > 50) // 灯条的长度差距
                    continue;
                // 中心矩形不符合比例的过滤
                float armour_wide = ((light1_distance + light2_distance) * 0.5f); // 算出灯条构成梯形的平均宽度
                if (((armour_length / armour_wide) < ARMOUR_PROPORTION_MIN) || ((armour_length / armour_wide) > ARMOUR_PROPORTION_MAX))
                    continue;
                // 用向量计算角度差
                cv::Point2f vec1 = light1.midPoints[1] - light1.midPoints[0]; // 灯条1向量
                cv::Point2f vec2 = light2.midPoints[1] - light2.midPoints[0]; // 灯条2向量
                float dot_product = abs(vec1.x * vec2.x + vec1.y * vec2.y); // 点积
                float magnitude1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y); // 向量1的模
                float magnitude2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y); // 向量2的模
                float angle_diff = acos(dot_product / (magnitude1 * magnitude2)) * (180.0f / CV_PI); // 计算夹角并转换为角度
                // debug
                // ROS_INFO("magnitude1: %.2f, magnitude2: %.2f", magnitude1, magnitude2);
                // ROS_INFO("angle_diff:%.2f", angle_diff);
                // ROS_INFO("dot_product:%.2f", dot_product);
                if (angle_diff > 12.0f) // 角度差过滤
                    continue;
                // 符合所有条件，绘制匹配线
                cv::Point2f center1 = (light1.midPoints[0] + light1.midPoints[1]) * 0.5f; // 灯条中心
                cv::Point2f center2 = (light2.midPoints[0] + light2.midPoints[1]) * 0.5f; // 灯条中心
                cv::line(frame, center1, center2, cv::Scalar(0, 0, 255), 2);
                cv::line(frame, light1.midPoints[0], light2.midPoints[1], cv::Scalar(0, 0, 255), 2);
                cv::line(frame, light2.midPoints[0], light1.midPoints[1], cv::Scalar(0, 0, 255), 2);
                cv::line(frame, light1.midPoints[0], light2.midPoints[0], cv::Scalar(0, 0, 255), 2);
                cv::line(frame, light2.midPoints[1], light1.midPoints[1], cv::Scalar(0, 0, 255), 2);
                std::array<cv::Point2f, 4> rect_point; // 静态数组储存四个点
                // 存入点进数组
                rect_point[0] = light1.midPoints[0];
                rect_point[1] = light1.midPoints[1];
                rect_point[2] = light2.midPoints[0];
                rect_point[3] = light2.midPoints[1];
                result_rect.push_back(rect_point); // 压入四个点的静态数组array
                // n = 2;
                // break;
            }
        }
        cv::imshow("debug", frame); // debug
    return result_rect; // 返回装甲板四个点
}

// 装甲板数字的扣图和仿射变换
cv::Mat armour_transform(std::array<cv::Point2f, 4>& array_rect, cv::Mat& image_raw){
    cv::Mat number_image; // 仿射变换后扣出的图片
    sortPointsClockwise(array_rect); // 顺时针排序四个点

    // 定义目标矩形区域（宽度和高度根据需求调整）
    int width = 32;  // 裁剪后图像的宽度
    int height = 28; // 裁剪后图像的高度
    std::array<cv::Point2f, 4> dst_points = {
        cv::Point2f(0, height),            // 左上
        cv::Point2f(width-1, height),      // 右上
        cv::Point2f(width-1, 0), // 右下
        cv::Point2f(0, 0)      // 左下
    };   
    // 计算仿射变换矩阵
    cv::Mat transform_matrix = cv::getPerspectiveTransform(array_rect, dst_points);
    cv::warpPerspective(image_raw, number_image, transform_matrix, cv::Size(width, height));

    return number_image;
}

// 对四边形的四个无序点进行按顺时针排序
void sortPointsClockwise(std::array<cv::Point2f, 4>& array_rect){
    cv::Point2f center(0,0);
    // 计算四个点的中心点
    for (const auto& pt : array_rect){
        center += pt;
    }
    center /= 4.0f;
    // 定义排序规则
    auto compare = [center](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center.y, a.x - center.x) < 
               std::atan2(b.y - center.y, b.x - center.x);
    };
    // 排序
    std::sort(array_rect.begin(), array_rect.end(), compare);
    // std::swap(array_rect[2], array_rect[3]); // 交换最后两个点
}
