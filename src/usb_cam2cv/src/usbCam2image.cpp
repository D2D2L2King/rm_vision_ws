#include "../include/usbCam2image.h"

int main(int argc, char *argv[]) {
    int video_num = 0; // 默认摄像头编号为0
    int camera_mode = 0; // 相机模式, 0表示不保存图像，1表示保存图像
    std::string video_string = "0"; // 默认摄像字符串
    
    // 识别传入的参数
    if (argc >= 2) 
    {
        video_string = argv[1];
        video_num = std::stoi(video_string); // 将字符串转换为整数
        video_string = "0";
    }

    if (argc >= 3)
    {
        video_string = argv[2];
        camera_mode = std::stoi(video_string); // 将字符串转换为整数
        video_string = "0";
    }
    // 初始化ROS节点
    ros::init(argc, argv, "usb_cam2image_node"); 
    ros::NodeHandle nh;
    // opencv打开usb摄像头
    cv::VideoCapture cap; // 0表示默认摄像头
    cap.open(video_num, cv::CAP_V4L2); // 打开默认摄像头
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); // 设置摄像头的宽度 1280
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // 设置摄像头的高度 720

    // 检查摄像头是否打开
    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video device");
        return -1;
    }
    cv::Mat frame, output; // 用于存储摄像头读取的图像帧

    ros::Publisher image_puber = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 10); // 创建图像发布者，订阅话题为 "image_raw"， 队列大小为10

    std_msgs::Header header; // 创建ROS消息头
    // 创建一个循环，持续读取摄像头数据
    ros::Rate loop_rate(120); // 设置循环频率为100Hz

    int photo_count = 0; // 用于计数拍摄的照片数量
    while (ros::ok()) 
    {
        // 从摄像头读取一帧图像
        cap >> frame;
        if (frame.empty()) // 检查是否成功读取图像
        {
            ROS_ERROR("Could not read frame from video device");
            continue; // 如果读取失败，跳过本次循环
        }

        float fps = cap.get(cv::CAP_PROP_FPS); // 获取摄像头的帧率
        output = frame.clone(); // 克隆当前帧到输出图像
        drawFPS(frame, fps); // 绘制FPS到图像

        // 显示图像
        cv::imshow("USB Camera Feed", frame);
        cv::waitKey(1); // 等待1毫秒以显示图像

        if (camera_mode == 1) // 如果相机模式为1，保存图像
        {
            if (32 == cv::waitKey(10))
            {
                // 如果按下空格键，保存当前图像
                if (photo_count >= 1000) // 如果照片数量超过1000，重置计数器
                {
                    photo_count = 0;
                }
                std::string filename = "../grab/photo_" + std::to_string(photo_count++) + ".jpg"; // 生成文件名
                cv::imwrite(filename, output); // 保存图像到文件
                ROS_INFO("Saved image: %s", filename.c_str()); // 打印保存的图像文件名
            }
        }

        header.stamp = ros::Time::now(); // 设置消息头的时间戳

        header.frame_id = "usb_cam_frame"; // 设置消息头的帧ID

        // 将OpenCV图像转换为ROS消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", output).toImageMsg();
        
        // 发布图像消息
        image_puber.publish(msg);

        loop_rate.sleep(); // 控制循环频率
        ros::spinOnce(); // 处理ROS回调函数
    }

    return 0;
}

