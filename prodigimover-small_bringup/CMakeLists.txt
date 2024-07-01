cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  prodigimover_msgs
  roscpp
  rosserial_python
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS 
 LIBRARIES prodigimover_bringup
 CATKIN_DEPENDS roscpp tf2 tf2_ros nav_msgs prodigimover_msgs geometry_msgs sensor_msgs std_msgs 
# DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(prodigimover_small_odom src/prodigimover_small_odom.cpp)
add_executable(prodigimover_small_odom_node src/prodigimover_small_odom_node.cpp)
target_link_libraries(prodigimover_small_odom_node  prodigimover_small_odom ${catkin_LIBRARIES})
add_dependencies(prodigimover_small_odom prodigimover_msgs_generate_messages_cpp)