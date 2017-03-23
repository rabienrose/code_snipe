CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(CoreDataLib)

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../FindPackage)
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(OpenCV 3.0.0 REQUIRED core highgui imgproc video features2d calib3d imgcodecs)
FIND_PACKAGE(Boost REQUIRED system serialization filesystem)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})


INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../dataStructure
  ${PROJECT_SOURCE_DIR}/../../bridge
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../Visualisation
  ${PROJECT_SOURCE_DIR}/../Common
  ${PROJECT_SOURCE_DIR}/../../3rdparty
  ${PROJECT_SOURCE_DIR}/Tool
  ${PROJECT_SOURCE_DIR}/../../basicLib/searchByProjection/
IF(APPLE)
  /Applications/MATLAB_R2016a.app/extern/include
ELSE()
  /usr/local/MATLAB/R2016a/extern/include/
ENDIF()
)

IF(APPLE)
set(LIBS
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/../../3rdparty/g2o/libg2o.dylib
  /Applications/MATLAB_R2016a.app/bin/maci64/libmat.dylib
  /Applications/MATLAB_R2016a.app/bin/maci64/libmx.dylib)
ELSE()
set(LIBS
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/../../3rdparty/g2o/libg2o.so
  /usr/local/MATLAB/R2016a/bin/glnxa64/libmwfl.so
  /usr/local/MATLAB/R2016a/bin/glnxa64/libmwi18n.so
  /usr/local/MATLAB/R2016a/bin/glnxa64/libut.so
  /usr/local/MATLAB/R2016a/bin/glnxa64/libboost_filesystem.so.1.56.0
  /usr/local/MATLAB/R2016a/bin/glnxa64/libmwcpp11compat.so
  

  /usr/local/MATLAB/R2016a/bin/glnxa64/libmat.so
  /usr/local/MATLAB/R2016a/bin/glnxa64/libmx.so
)
ENDIF()

ADD_LIBRARY(${PROJECT_NAME} SHARED
  coreData.cpp
  coreData.hpp
  ../dataStructure/Frame.cpp
  ../dataStructure/Frame.hpp
  ../dataStructure/GlobalMap.cpp
  ../dataStructure/GlobalMap.hpp
  ../dataStructure/MapPoint.cpp
  ../dataStructure/MapPoint.hpp
  ../dataStructure/TypeDef.hpp
  ../dataStructure/TypeDef.cpp

  ../Visualisation/ViewInterfaceImp.h
  ../Visualisation/ViewInterfaceImp.cpp
  ../Common/Utils.cpp
  ../Common/Utils.h
  ../../basicLib/searchByProjection/FeatureTool.hpp
  ../../basicLib/searchByProjection/FeatureTool.cpp
  ../../bridge/ReadGlobalMapFromM.cpp
  ../../bridge/ReadGlobalMapFromM.hpp
  ../../bridge/SaveGlobalMapToM.cpp
  ../../bridge/SaveGlobalMapToM.hpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS})
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR} )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR} )

link_directories(/usr/local/MATLAB/R2016a/bin/glnxa64/)
target_link_libraries(${PROJECT_NAME} -L/usr/local/MATLAB/R2016a/bin/glnxa64/libmat.so -L/usr/local/MATLAB/R2016a/bin/glnxa64/libmwfl.so)
