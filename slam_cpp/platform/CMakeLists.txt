CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(core_slam)

# Check C++11 or C++0x support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -Wall -march=native -g")
   ADD_DEFINITIONS(-DCOMPILEDWITHC11)
   MESSAGE(STATUS "Using flag -std=c++11.")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   ADD_DEFINITIONS(-DCOMPILEDWITHC0X)
   MESSAGE(STATUS "Using flag -std=c++0x.")
ELSE()
   MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../FindPackage)
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(OpenCV 3.0.0 REQUIRED core highgui imgproc video features2d calib3d imgcodecs xfeatures2d)
FIND_PACKAGE(Boost REQUIRED system serialization filesystem random)
FIND_PACKAGE(Pangolin REQUIRED)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set(COMMON_OUTPUT "${CMAKE_CURRENT_LIST_DIR}/../../../../common/dist/x64")

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure
  ${CMAKE_CURRENT_LIST_DIR}/../../bridge
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/opticalFlow
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/ORBExtractor
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByProjection/
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/triangulate
  ${CMAKE_CURRENT_LIST_DIR}/../Visualisation
  ${CMAKE_CURRENT_LIST_DIR}/../../config
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/optimizePose
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByEpipolar
  ${CMAKE_CURRENT_LIST_DIR}/../Common
  ${CMAKE_CURRENT_LIST_DIR}/../XmlSLAM
  ${CMAKE_CURRENT_LIST_DIR}/../../3rdparty
  ${CMAKE_CURRENT_LIST_DIR}/Tool
  ${CMAKE_CURRENT_LIST_DIR}/optimizer
  ${CMAKE_CURRENT_LIST_DIR}/../../../../common/include
)

IF(APPLE)
set(LIBS
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/g2o/libg2o.dylib
  ${COMMON_OUTPUT}/lib/libroadDBComm.dylib
)
ELSE()
set(LIBS
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/g2o/libg2o.so
  ${COMMON_OUTPUT}/lib/libroadDBComm.so
)
ENDIF()
message("libs: ${LIBS}")


OPTION(USE_SGD_SWITCH "Option description" ON)
IF(USE_SGD_SWITCH MATCHES ON)
  ADD_DEFINITIONS(-DUSE_SGD)
ENDIF()

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/Frame.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/Frame.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/GlobalMap.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/GlobalMap.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/MapPoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/MapPoint.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/TypeDef.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../dataStructure/TypeDef.cpp

  ${CMAKE_CURRENT_LIST_DIR}/Platform.hpp
  ${CMAKE_CURRENT_LIST_DIR}/Platform.cpp

  ${CMAKE_CURRENT_LIST_DIR}/Tool/utils.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Tool/GMapHelper.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Tool/paramParser.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Tool/outPut.cpp
  ${CMAKE_CURRENT_LIST_DIR}/tracker/Tracker.cpp
  ${CMAKE_CURRENT_LIST_DIR}/mapping/Mapping.cpp
  ${CMAKE_CURRENT_LIST_DIR}/initializer/MatchByOpticFlow.cpp
  ${CMAKE_CURRENT_LIST_DIR}/initializer/tryInitPose.cpp
  ${CMAKE_CURRENT_LIST_DIR}/optimizer/OptimizeByLocalBA.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Tool/Fill2DMatches.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Tool/Visualization.cpp

  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/opticalFlow/optflowIni.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/opticalFlow/kd_tree.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/opticalFlow/optflow.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/opticalFlow/rt.cpp

  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/ORBExtractor/RaulORB.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/ORBExtractor/RaulORB.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/ORBExtractor/ORBextractor.cc
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/ORBExtractor/ORBextractor.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/FeatureExtractor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/FeatureExtractor.hpp
  #SGD related file.
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/Config.cc
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/Config.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/ISlamConfig.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDDescriptor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDDescriptor.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDExtractor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDExtractor.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDExtractor.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDWrapper.cc
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/SGDWrapper.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/vehicleCfg.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/vehicleCfg.h
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/featureExtractor/SGD/typeDef.h
  

  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/optimizePose/Optimizer.cc
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/optimizePose/Optimizer.h

  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByProjection/FeatureTool.hpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByProjection/FeatureTool.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByProjection/searchByProjection.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByProjection/searchByProjection.hpp

  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByEpipolar/searchByEpipolarHandle.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/searchByEpipolar/searchByEpipolarHandle.hpp

  
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/triangulate/TriangulateHandle.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../../basicLib/triangulate/TriangulateHandle.hpp

  ${CMAKE_CURRENT_LIST_DIR}/../Visualisation/ViewInterfaceImp.h
  ${CMAKE_CURRENT_LIST_DIR}/../Visualisation/ViewInterfaceImp.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../Common/Utils.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../Common/Utils.h
  ${CMAKE_CURRENT_LIST_DIR}/../Common/NewMPCreator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../Common/NewMPCreator.h
  ${CMAKE_CURRENT_LIST_DIR}/../Common/comAglo.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../Common/comAglo.h
  ${CMAKE_CURRENT_LIST_DIR}/../Common/AlgoInterfaceImp.cpp
)

target_link_libraries(${PROJECT_NAME} ${LIBS})
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR} )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR} )
