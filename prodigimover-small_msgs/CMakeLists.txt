cmake_minimum_required(VERSION 3.0.2)
project(prodigimoversmall_msgs)



find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Velocities.msg
  PID.msg
  Imu.msg
  MagneticField.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)


catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
