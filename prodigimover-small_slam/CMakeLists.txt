cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_slam)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prodigimover-small_slam
#  CATKIN_DEPENDS sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
