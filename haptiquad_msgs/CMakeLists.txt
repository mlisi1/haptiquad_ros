cmake_minimum_required(VERSION 2.8.3)
project(haptiquad_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  ResidualsStamped.msg
  ObserverGains.msg
  EstimatedForces.msg
  ResidualErrorStamped.msg
  FrictionParameters.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)