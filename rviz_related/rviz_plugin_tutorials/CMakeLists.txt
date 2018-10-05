cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_tutorials)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS cv_bridge rviz geometry_msgs sensor_msgs veh_msg)

catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # UIC output directory
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
#set(CMAKE_AUTOMOC ON) # cmake will auto moc but we have used qt4_wrap_cpp, so this should not be enabled
set(CMAKE_AUTOUIC ON)
add_definitions(-DQT_NO_KEYWORDS)
IF(DEFINED ENV{DEV_BUILD})
  ADD_DEFINITIONS(-DDEV_BUILD)
  set(HEADER_FILES
          src/RemoteControl.h
          src/imu_display.h
          src/teleop_panel.h
          src/image_widget.h
          src/my_panel.h
          src/record_play.h
          )
  set(QT_RESOURCES
          #src/my_panel.qrc
          )
  qt4_add_resources(
          QT_RESOURCES_CPP
          ${QT_RESOURCES}
  )

  qt4_wrap_cpp(
          MOC_FILES
          ${HEADER_FILES}
  )

  qt4_wrap_ui(
          UIC_FILES
          src/my_panel.ui
  )

  set(SOURCE_FILES
          src/my_panel.cpp
          src/imu_display.cpp
          src/RemoteControl.cpp
          src/remoteConHelper.cpp
          src/teleop_panel.cpp
          src/image_widget.cpp
          src/record_play.cpp
          src/context.cpp
          )
ELSE()
  set(HEADER_FILES
          src/imu_display.h
          src/teleop_panel.h
          src/image_widget.h
          src/my_panel.h
          src/record_play.h
          )
  set(QT_RESOURCES
          #src/my_panel.qrc
          )
  qt4_add_resources(
          QT_RESOURCES_CPP
          ${QT_RESOURCES}
  )

  qt4_wrap_cpp(
          MOC_FILES
          ${HEADER_FILES}
  )

  qt4_wrap_ui(
          UIC_FILES
          src/my_panel.ui
  )

  set(SOURCE_FILES
          src/my_panel.cpp
          src/imu_display.cpp
          src/remoteConHelper.cpp
          src/teleop_panel.cpp
          src/image_widget.cpp
          src/record_play.cpp
          src/context.cpp
          )
ENDIF()

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${QT_RESOURCES_CPP} ${MOC_FILES} ${UIC_FILES})
add_dependencies(${PROJECT_NAME}  loc_server_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


## Install rules

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY conf/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)

install(DIRECTORY icon/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icon)

install(DIRECTORY media/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)


