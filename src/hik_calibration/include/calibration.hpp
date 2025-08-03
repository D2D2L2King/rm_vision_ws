#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "debug_tools.hpp"
using namespace std;
using namespace cv;

// 默认的相机内参和畸变系数
Mat default_camera_matrix = (cv::Mat_<double>(3, 3) << 506.6888704186833, 0, 345.7123633736691,
                             0, 507.9811936572676, 265.2223892590567,
                             0, 0, 1);
Mat default_distcoeff_matrix = (cv::Mat_<double>(1, 5) << -0.434035765932086, 0.2054734470000142,
                                0.001976050532605929, 0.007128262298018064, -0.05822401481469553);
class PnP
{
public:
  PnP();
  PnP(std::string file_path, bool Isimshow = true, cv::Size chessboard_size = cv::Size(8, 6),
      cv::Size img_size = cv::Size(640, 640), float real_dis = 0.022);
  // 图像去畸变
  void indistortion(Mat &src, Mat &dst);
  // 获取旋转矩阵和平移矩阵 这个是通用模板因此需要输入objectPoints 后续可能会根据实际参数将objectPoints固定根据需求选择
  bool get_R_T(std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints, cv::Mat &R, cv::Mat &T);
  // 绘制三维坐标轴
  void draw_axis(Mat &img, cv::Mat &Rvec, Mat &Tvec);

private:
  void camera_calibration(std::string file_path, bool Isimshow = true,
                          cv::Size chessboard_size = cv::Size(8, 6), cv::Size img_size = cv::Size(640, 640), float real_dis = 0.022);
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
};
/*
  @brief:相机标定
  @param:file_path:标定图片路径
          Isimshow:是否显示标定结果
          chessboard_size:棋盘格大小
          img_size:图像大小
          real_dis:棋盘格实际距离
*/
void PnP::camera_calibration(std::string file_path, bool Isimshow, cv::Size chessboard_size, cv::Size img_size, float real_dis)
{
  /*
    使用 cv::glob函数读取文件夹中的所有图像
  */
  vector<String> chessboards;
  std::string pattern = file_path + "/*.jpg";
  glob(pattern, chessboards);
  /*
    设置世界坐标系的角点坐标
    每一个角点0.022m
  */
  vector<Point3f> object_points;
  for (int i = 0; i < chessboard_size.height; i++)
  {
    for (int j = 0; j < chessboard_size.width; j++)
    {
      object_points.push_back(Point3f(j * real_dis, i * real_dis, 0));
    }
  }
  // 创建数组储存所有图像的像素角点坐标和世界坐标系角点坐标
  vector<vector<Point2f>> image_points;
  vector<vector<Point3f>> object_points_all;
  for (const auto &chessboard_path : chessboards)
  {
    // 角点优化函数必须使用单通道图像，所以优化前需要将图像转换为灰度图像 在绘制时又切换会彩色就能获得彩色角点
    // 创建保存角点的数组 必须是2f
    Mat chessboard = imread(chessboard_path);
    vector<Point2f> corners;
    // 寻找角点
    // 第四个参数表示使用自适应阈值方法来处理图像。自适应阈值是一种局部阈值方法，它根据图像中每个
    // 像素的邻域亮度来确定该像素是否应该被视为前景或背景
    // 第五个参数表示图像应该被归一化到0-255的范围内。归一化图像可以减少光照变化对角点检测的影响
    bool flag = findChessboardCorners(chessboard, chessboard_size, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    if (!flag)
    {
      std::cout << "\033[33m" << "make sure you set the right chessboard size!!!" << "\033[0m" << std::endl;
      std::cout << chessboard_path << std::endl;
      continue;
    }
    /*对角点进行亚像素精确化
    这个参数定义了迭代过程的停止准则。它是一个包含三个元素的元组 (type, maxCount, epsilon)，其中：
    type 是停止条件的类型，可以是 cv2.TERM_CRITERIA_EPS（达到指定的精度 epsilon）或 cv2.TERM_CRITERIA_MAX_ITER（达到最大迭代次数 maxCount）或两者的组合。
    maxCount 是最大迭代次数。
    epsilon 是所需的精度。
    */
    cvtColor(chessboard, chessboard, COLOR_BGR2GRAY);
    cornerSubPix(chessboard, corners, Size(5, 5), Size(-1, -1),
                 TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
    if (Isimshow)

    {
      cvtColor(chessboard, chessboard, COLOR_GRAY2BGR);
      drawChessboardCorners(chessboard, chessboard_size, corners, true);
      imshow("chessboard", chessboard);
      waitKey(10);
    }
    image_points.push_back(corners);
    object_points_all.push_back(object_points);
  }
  // 相机内参，畸变系数，旋转矩阵和平移向量
  cv::Mat R, T;
  calibrateCamera(object_points_all, image_points, img_size, cameraMatrix, distCoeffs, R, T);
  cout << "cameraMatrix:\n"
       << cameraMatrix << endl;
  cout << "distCoeffs:\n"
       << distCoeffs << endl;
}
PnP::PnP() : cameraMatrix(default_camera_matrix), distCoeffs(default_distcoeff_matrix) {};
PnP::PnP(std::string file_path, bool Isimshow, cv::Size chessboard_size, cv::Size image_size, float real_dis)
{
  camera_calibration(file_path, Isimshow, chessboard_size, image_size, real_dis);
}

void PnP::indistortion(Mat &src, Mat &dst)
{
  cv::undistort(src, dst, cameraMatrix, distCoeffs);
}

/*
  根据获取的四个角点坐标计算相机R T
  计算出来的T
*/
bool PnP::get_R_T(std::vector<Point3f> &object_points, std::vector<Point2f> &image_points, Mat &reslut_R, Mat &reslut_T)
{
  if (object_points.size() != image_points.size())
  {
    debug_tools::printError("make sure the size of object_points and image_points is equal");
    return false;
  }
  if (object_points.size() < 4)
  {
    debug_tools::printError("make sure the size of object_points is lager than 4!");
    return false;
  }
  return cv::solvePnP(object_points, image_points, cameraMatrix, distCoeffs, reslut_R, reslut_T, false, cv::SOLVEPNP_IPPE);
  // return cv::solvePnP(object_points,image_points,cameraMatrix,distCoeffs,reslut_R,reslut_T);
}

void PnP::draw_axis(Mat &src, Mat &R_vec, Mat &T_vec)
{
  // x,y,z坐标轴
  std::vector<Point3f> axis;
  axis.push_back(Point3f(0, 0, 0));
  axis.push_back(Point3f(0.1, 0, 0));
  axis.push_back(Point3f(0, 0.1, 0));
  axis.push_back(Point3f(0, 0, 0.1));
  // 投影
  std::vector<Point2f> imag_points;
  cv::projectPoints(axis, R_vec, T_vec, cameraMatrix, distCoeffs, imag_points);
  // 绘制图像
  // x轴
  cv::line(src, imag_points[0], imag_points[1], Scalar(0, 0, 255), 3);
  cv::putText(src, "x", imag_points[1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 4);
  // y轴
  cv::line(src, imag_points[0], imag_points[2], Scalar(0, 255, 0), 3);
  cv::putText(src, "y", imag_points[2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 4);
  // z轴
  cv::line(src, imag_points[0], imag_points[3], Scalar(255, 0, 0), 3);
  cv::putText(src, "z", imag_points[3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 4);
}