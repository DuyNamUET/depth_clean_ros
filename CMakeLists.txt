cmake_minimum_required(VERSION 2.8.3)
project(depth_clean_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  realsense2_camera
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
)

## OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(clean_depth src/clean_depth.cpp)
target_link_libraries(clean_depth ${catkin_LIBRARIES} ${OpenCV_LIBS})