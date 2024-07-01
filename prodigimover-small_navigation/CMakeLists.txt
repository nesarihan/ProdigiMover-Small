cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  amcl
  geometry_msgs
  move_base
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prodigimover_navigation
#  CATKIN_DEPENDS amcl move_base
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



#add_executable(send_goals src/send_goals.cpp)
#target_link_libraries(send_goals ${catkin_LIBRARIES})