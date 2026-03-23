cmake_minimum_required(VERSION 2.8.3)
project(haptiquad_plot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)



catkin_package()

# Mark Python scripts as executable
catkin_install_python(PROGRAMS
  scripts/force_plotter.py
  scripts/residual_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install Python modules
catkin_python_setup()