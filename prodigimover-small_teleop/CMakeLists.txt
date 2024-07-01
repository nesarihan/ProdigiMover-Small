cmake_minimum_required(VERSION 3.0.2)
project(prodigimover-small_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  sensor_msgs

)


catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prodigimover_teleop
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs rospy
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/prodigimover_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#add_library(teleop_twist_joy  src/teleop_twist_joy)
#target_link_libraries(teleop_twist_joy ${catkin_LIBRARIES})
#add_executable(teleop_node src/teleop_node.cpp)
#add_executable(f710_teleop_joy_node src/f710_teleop_joy_node.cpp)
#target_link_libraries(teleop_node teleop_twist_joy ${catkin_LIBRARIES})
#add_dependencies(teleop_twist_joy prodigimover_msgs_generate_messages_cpp)
#add_dependencies(f710_teleop_joy_node prodigimover_msgs_generate_messages_cpp)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  
#install(TARGETS teleop_twist_joy teleop_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/teleop_twist_joy
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)



install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)