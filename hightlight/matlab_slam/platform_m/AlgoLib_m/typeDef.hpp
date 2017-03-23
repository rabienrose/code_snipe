//
//  typeDef.hpp
//
//  Created by zhaiq on 7/6/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#pragma once

#include <mex.h>
#include <opencv2/opencv.hpp>

//#define DEBUG_MESSAGE_PRINT

#ifdef DEBUG_MESSAGE_PRINT
#   define MESSAGE_PRINT(...)   mexPrintf(__VA_ARGS__)
#else
#   define MESSAGE_PRINT(...)
#endif

#define ERROR_LOG(...)       mexPrintf(__VA_ARGS__)

#define DEFAULT_OUTPUT(n, p) \
                            for(int i=0; i<n; i++) { \
                                p[i] = mxCreateDoubleScalar(1.0);\
                            }

namespace ygomi {

    // Each track item is used to find the relationship bewteen map point and its key point,
    // it gives the information that the key point(2D) is in which frame, and which key.
    // NOTE!!! To obtain better understanding, we abandon memory layout.
    typedef struct tagTrack
    {
        long frameId;    // FrameId shows that the key point is in which frame.
        int  keyPointId; // Which key point of frame.
    } Track;
    
    typedef struct tagMapPoint
    {
        bool                      isBad;      // Map point status, if it is set to true, current map point is discarded.
        std::vector<ygomi::Track> tracks;     // Each map point(3D) is projected into multiple key points(2D),
                                              // those key points list is defined as the track of current map point.
        cv::Point3f               pt;         // Map point position.
        cv::Mat                   descriptor; // Map point descriptor, a merged descriptor from all tracks,
                                              // which is defined as a 1 x N array, and N is the dimension of descriptor.
    } MapPoint;
    
    // Abandon memory layout to obtain better understanding.
    typedef struct tagKeyPoint
    {
        long         mapPointId; // Which map point that key point is corresponding to.
        cv::KeyPoint pt;         // Key point position.
        cv::Mat      descriptor; // Key point descriptor, the same difinition with MapPoint area.
    } KeyPoint;
    
    typedef struct tagFrame
    {
        long                         frameId;
        int                          status;    // Whether frame is a key frame.
        std::vector<ygomi::KeyPoint> keyPoints; // All key point in frame.
        cv::Mat                      pose;      // Camera pose of frame, which is defined as a 3 x 4 float matrix.
    } Frame;
    
//    typedef tagCoreData
//    {
//        std::vector<Frame>      frameList;      // All processed frames.
//        std::vector<MapPoint>   mapPointList;   // All map points.
//    } CoreData;
}

