cmake_minimum_required(VERSION 3.0.2)
project(darknet_ros_3d_trial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gb_visual_detection_3d_msgs
  roscpp
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(darknet3d_trial_node src/darknet3d_trial_node.cpp)
target_link_libraries(darknet3d_trial_node ${catkin_LIBRARIES})
