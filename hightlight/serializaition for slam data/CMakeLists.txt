CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ViewInterface)

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
ENDIF(APPLE)

FIND_PACKAGE(Boost REQUIRED filesystem serialization)

INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}
#${PROJECT_SOURCE_DIR}/../../server/include
${PROJECT_SOURCE_DIR}/../../vehicle/include
${Boost_INCLUDE_DIRS}

${PROJECT_SOURCE_DIR}/../../common/Thirdparty/ORB_SLAM2/include
${PROJECT_SOURCE_DIR}/../../common/
${PROJECT_SOURCE_DIR}/../../common/include
${PROJECT_SOURCE_DIR}/../../common/Thirdparty
${PROJECT_SOURCE_DIR}/../../common/visualisation
)

#link_directories(${PROJECT_SOURCE_DIR}/../../server/lib
#${PROJECT_SOURCE_DIR}/../../vehicle/lib)

set(LIBS
  ${Boost_LIBRARIES}
  #algoServer
  #algoVehicle
)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(${PROJECT_NAME} SHARED
		ViewInterfaceImp.cpp
		ViewInterfaceImp.h
		ViewILibMain.cpp
    #${PROJECT_SOURCE_DIR}/../../common/source/AlgoInterfaceImp.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIBS})


