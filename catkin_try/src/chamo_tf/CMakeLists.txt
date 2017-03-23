cmake_minimum_required(VERSION 2.8.3)
project(chamo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sensor_msgs
  std_msgs
)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
add_executable(champ_tf src/chamo_tf.cpp)
target_link_libraries(champ_tf ${catkin_LIBRARIES})