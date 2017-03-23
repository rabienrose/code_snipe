CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(test_voting)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
ADD_DEFINITIONS(-DUSE_SGD)
ADD_DEFINITIONS(-DENABLE_SPTR_INSLAM)

# Check C++11 or C++0x support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   ADD_DEFINITIONS(-DCOMPILEDWITHC11)
   MESSAGE(STATUS "Using flag -std=c++11.")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   ADD_DEFINITIONS(-DCOMPILEDWITHC0X)
   MESSAGE(STATUS "Using flag -std=c++0x.")
ELSE()
   MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

IF(APPLE)
  ADD_DEFINITIONS(-DAPPLE)
  MESSAGE(STATUS "Compiling on Mac")
ENDIF()

FIND_PACKAGE(OpenCV 3.0.0 REQUIRED)
FIND_PACKAGE(Boost REQUIRED filesystem system serialization thread)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../../../common/dist/x64/include
    ${PROJECT_SOURCE_DIR}/../../common/include
    ${PROJECT_SOURCE_DIR}/../../../common/include
    ${PROJECT_SOURCE_DIR}/../../loc/SLAM/include
    ${PROJECT_SOURCE_DIR}/../../loc/SLAM/ORB_SLAM2/include
    ${PROJECT_SOURCE_DIR}/../../common/Thirdparty/ORB_SLAM2/include
    ${PROJECT_SOURCE_DIR}/../../common
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../../common/Thirdparty/Sophus/sophus
)

LINK_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/../../../common/dist/x64/lib
    ${PROJECT_SOURCE_DIR}/../../loc/lib
    ${PROJECT_SOURCE_DIR}/../../common/Thirdparty/DBoW2/lib
    ${PROJECT_SOURCE_DIR}/../../common/Thirdparty/g2o/lib
)

SET(LINKED_LIBS
    algoSlamV
    rdbLogWrapper
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

ADD_EXECUTABLE(test_voting
  main.cpp
  main_helper.h
  pose_voting.h
  pose_voting.cpp
  ${PROJECT_SOURCE_DIR}/../../common/source/AlgoInterfaceImp.cpp
  ${PROJECT_SOURCE_DIR}/../../common/visualisation/ViewInterfaceImp.cpp
)
TARGET_LINK_LIBRARIES(test_voting ${LINKED_LIBS})
