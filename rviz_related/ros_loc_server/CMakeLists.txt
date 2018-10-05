cmake_minimum_required(VERSION 2.8.3)
project(loc_server)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  message_generation
  cv_bridge
  dynamic_reconfigure
)
IF(DEFINED ENV{DEV_BUILD})
  ADD_DEFINITIONS(-DDEV_BUILD) 
ENDIF()

add_message_files(DIRECTORY msg
FILES  
loc_predict_msg.msg
loc_update_msg.msg
match_info_msg.msg
server_img_msg.msg
rviz_control_msg.msg
reproj_err_msg.msg
view_control_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

FIND_PACKAGE(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
message(STATUS ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

add_executable(loc_server src/main.cpp ../rviz_plugin_tutorials/src/context.cpp)
add_dependencies(loc_server  loc_server_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(loc_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
 )


