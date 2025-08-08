#include "../include/process.h"

#define MAX_FPS 150 // 默认最大帧率
#define AM_BLUE 0 // 装甲板颜色标志，0为蓝色，1为红色
#define AM_RED 1 // 装甲板颜色标志，0为蓝色，1为红色

void call_back(const sensor_msgs::ImageConstPtr& msg); // 回调函数声明

std::mutex image_mutex;  // 保护共享图像的互斥锁
cv::Mat image_tmp, image_processed; // 用于存储处理后的图像, 全局变量

int main(int argc, char* argv[]) {
    bool color_select = AM_BLUE; // 瞄准颜色标志, 默认为蓝色
    // 处理传入参数
    if (argc >= 2)
    {
        std::string aim_color = argv[1];
        if (aim_color == "R") // 如果瞄准红色
            color_select = AM_RED; // 设置为红色
        else if (aim_color == "B") // 如果瞄准蓝色
            color_select = AM_BLUE; // 设置为蓝色
    }
    ros::init(argc, argv, "tradtion_process_image_node"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建ROS节点句柄
    ros::Subscriber suber = nh.subscribe<sensor_msgs::Image>("/hik2cv/hik_image", 3, call_back); // 订阅图像话题，队列大小为5
    ros::Publisher puber = nh.advertise<sensor_msgs::Image>("/hik2cv/processed_image", 2); // 发布处理后的图像话题
    int real_fps = MAX_FPS; // 默认最大帧率
    // 动态调整从参数服务器获取参数
    ros::Rate loop_rate(150); // 设置循环频率
    
    while (ros::ok()) 
    {
        // 从参数服务器获取参数
        
        nh.param("/hik2cv_node/realtime_fps", real_fps, 150); // 从参数服务器获取最大帧率，默认值为150
        ros::Rate loop_rate(real_fps); // 设置循环频率
        cv::Mat image_cv; // 用于显示处理后的图像
        {
            std::lock_guard<std::mutex> lock(image_mutex); // 加锁，保护共享图像
            if (!image_tmp.empty()) 
            {
                image_cv = image_tmp.clone(); // 深拷贝共享图像
            }
        }
        /****** 图像处理 ******/
        if (!image_cv.empty()) {
            auto lights_counter = image_processing(image_cv, color_select); // 调用图像处理函数
            // 输出检测到的灯条数量
            ROS_INFO("Detected %zu lights.", lights_counter.size()); // 输出检测到的灯条数量
            // 调用仿射变换函数
            if (lights_counter.size() != 0) // 当检测到装甲板时
            {
                image_processed = armour_transform(lights_counter[0], image_cv);
                cv::imshow("armour_number", image_processed);
            }

            
            
            


















        }
        /*********************/
        if (image_cv.empty()) {
            ROS_ERROR("No image received yet.");
        }
        else
        {
            // cv::imshow("result", image_processed); // debug
            cv::waitKey(1); // 等待1毫秒
        }

        std_msgs::Header header; // 创建ros消息头
        header.stamp = ros::Time::now(); // 设置时间戳
        header.frame_id = "hik2cv_frame"; // 设置帧ID
        // sensor_msgs::Image msg = *cv_bridge::CvImage(header, "bgr8", image_processed).toImageMsg(); // 将OpenCV图像转换为ROS消息格式
        // puber.publish(msg); // 发布图像消息


        ros::spinOnce(); // 处理回调函数
        loop_rate.sleep(); // 控制循环频率
    }

    
    cv::destroyAllWindows(); // 销毁所有OpenCV窗口
    return 0;
}

void call_back(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat temp = image2cv(msg); // 将ROS图像消息转换为OpenCV格式
    {
        // 加锁更新共享图像
        std::lock_guard<std::mutex> lock(image_mutex);
        image_tmp = temp.clone();  // 确保深拷贝
    }
    if (image_tmp.empty()) // 检查转换是否成功
    {
        ROS_ERROR("Failed to convert image from ROS to OpenCV format.");
        return; // 如果转换失败，直接返回
    }
}