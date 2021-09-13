cmake_minimum_required(VERSION 3.0.2)
project(rqt_herox_dashboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package()
catkin_python_setup()
catkin_install_python(PROGRAMS
  scripts/rqt_herox_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

