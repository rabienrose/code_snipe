cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")
project(show_map)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  message_generation
)

add_service_files(DIRECTORY srv
FILES
algo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

FIND_PACKAGE(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
message(STATUS ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
add_executable(show_map src/main.cpp ../rviz_plugin_tutorials/src/context.cpp)
add_dependencies(${PROJECT_NAME}  show_map_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

catkin_package(
  )




set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
 )

install(DIRECTORY paints_res/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paints_res)




#set(PKG_NAME ${PROJECT_NAME})
#set(PKG_LIBRARIES ${PROJECT_NAME})
#set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-config.cmake")
#configure_file("${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
#install(FILES ${CMAKE_BINARY_DIR}/${cmake_conf_file}
#    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake/ COMPONENT cmake)

#SET_DIRECTORY_PROPERTIES(PROPERTIES
#    ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/console_bridge-config.cmake
#    ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/console_bridge.pc)
