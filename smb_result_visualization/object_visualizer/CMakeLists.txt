cmake_minimum_required(VERSION 3.0.2)
project(object_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

# find_package(PCL 1.10 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_visualizer
  CATKIN_DEPENDS roscpp visualization_msgs
  # DEPENDS PCL
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(object_visualizer_node src/object_visualizer_node.cpp)
add_executable(cluster_visualizer_node src/cluster_visualizer_node.cpp)
add_executable(object_clustering_node src/object_clustering_node.cpp)



## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(object_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cluster_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_clustering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(object_visualizer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(cluster_visualizer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(object_clustering_node
  ${catkin_LIBRARIES}
)

target_link_libraries(object_clustering_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)