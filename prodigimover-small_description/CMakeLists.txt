cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prodigimover_description
   CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

include_directories( include 

  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)