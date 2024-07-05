cmake_minimum_required(VERSION 3.0.2)
project(pcd_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_publisher src/pcd_publisher.cpp)
target_link_libraries(pcd_publisher ${catkin_LIBRARIES})
