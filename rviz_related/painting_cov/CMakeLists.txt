cmake_minimum_required(VERSION 2.8.3)
project(painting_cov)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        visualization_msgs
        sensor_msgs
        std_msgs
        tf2_ros
        show_map
        message_generation
        )
add_service_files(DIRECTORY srv
        FILES
        algo.srv
        )
catkin_package(
)

FIND_PACKAGE(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PCL REQUIRED)

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/devel/include)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main_forCC_BoundaryPaint_solid_dashed.cpp)

add_dependencies(${PROJECT_NAME}  show_map_gencpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

