cmake_minimum_required(VERSION 2.8)
project(try_img)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(COMMON_SOURCE_FILES
        main.cpp
      )
add_executable(try_img ${COMMON_SOURCE_FILES})

find_package(Eigen3 REQUIRED)
include_directories (AFTER "${EIGEN3_INCLUDE_DIR}")
find_package(OpenCV REQUIRED)
include_directories (AFTER "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(try_img ${OpenCV_LIBS})
