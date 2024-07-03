cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  sensor_msgs
  std_msgs
)



# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)



include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}

)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}

)


catkin_package(
  CATKIN_DEPENDS gazebo_ros geometry_msgs  sensor_msgs std_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}

)


install(TARGETS 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)