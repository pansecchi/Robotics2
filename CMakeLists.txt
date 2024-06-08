cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  tf
  tf2_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf tf2_ros 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
