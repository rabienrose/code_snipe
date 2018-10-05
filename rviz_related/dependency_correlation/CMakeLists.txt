cmake_minimum_required(VERSION 2.8.3)
project(dependency_correlation)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dependency_correlation 
  record_msg
  loc_server
  rviz_plugin_tutorials
  rviz_animated_view_controller
  veh_msg
  show_map
)

set(CATKIN_PACKAGE_BIN_DESTINATION bin/)
install(PROGRAMS script/run.sh 
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

