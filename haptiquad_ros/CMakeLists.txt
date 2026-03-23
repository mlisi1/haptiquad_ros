cmake_minimum_required(VERSION 2.8.3)
project(haptiquad_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-error=unused-variable)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set(DEPS
  roscpp
  rospy
  pinocchio
  Eigen3
  std_msgs
  haptiquad_msgs
  haptiquad
  sensor_msgs
  nav_msgs
  message_filters
  anymal_msgs
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  haptiquad_msgs
  haptiquad
  anymal_msgs
)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs message_filters haptiquad_msgs haptiquad
  DEPENDS pinocchio
)

add_executable(haptiquad_anymal 
    src/anymal_wrapper.cpp
    src/wrapper_base.cpp    
)
target_link_libraries(haptiquad_anymal
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
)
target_include_directories(haptiquad_anymal PRIVATE
    include
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
install(
    TARGETS haptiquad_anymal
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add test dependencies here if needed
endif()
