cmake_minimum_required(VERSION 3.0.2)
project(traditional_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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
#  LIBRARIES traditional_vision
#  CATKIN_DEPENDS opencv sensor_msgs std_msgs
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

# 添加可执行文件
add_executable(process_image src/process_image.cpp include/image2cv.cpp)
# 链接catkin和OpenCV库
target_link_libraries(process_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
#############
## Install ##
#############

#############
## Testing ##
#############

