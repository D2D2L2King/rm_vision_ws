#include <iostream>
#include "calibration.hpp"
using namespace std;
using namespace cv;
string photo_dir_path = "/home/h1r0/rm_vision_ws/src/hik_calibration/calibration";
int main()
{
    PnP pnp(photo_dir_path, true, cv::Size(11,8), cv::Size(1440,1080));
    return 0;
}
