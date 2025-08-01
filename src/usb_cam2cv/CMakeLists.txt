cmake_minimum_required(VERSION 3.0.2)
project(usb_cam2cv)


find_package(catkin REQUIRED COMPONENTS # 查找 catkin 包
  geometry_msgs
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED) # 查找 OpenCV 包

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usb_cam2cv
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( # 添加头文件搜索路径
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(usbCam2image src/usbCam2image.cpp include/function.cpp)
# 链接catkin和OpenCV库
target_link_libraries(usbCam2image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
