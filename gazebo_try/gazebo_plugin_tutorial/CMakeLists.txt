cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(camp SHARED camp.cpp)
target_link_libraries(camp ${GAZEBO_LIBRARIES})

add_library(imup SHARED imup.cpp)
target_link_libraries(imup ${GAZEBO_LIBRARIES})

add_library(gpsp SHARED gpsp.cpp)
target_link_libraries(gpsp ${GAZEBO_LIBRARIES})
