//
//  SeacrhByEpipolarHandle.hpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/4/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#ifndef SeacrhByEpipolarHandle_hpp
#define SeacrhByEpipolarHandle_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

struct FrameInfo
{
    std::vector<cv::KeyPoint>   keys;
    cv::Mat                     descriptors;
    cv::Mat                     pose;
    float                       mediaDepth;
};

void searchByEpipolar(const FrameInfo& currUnmatched,
                      const FrameInfo& referUnmacthed,
                      const cv::Mat& K,
                      const std::vector<float>& scaleFactors,
                      std::vector<std::pair<int, int> >& matchedPairs,
                      std::vector<int8_t>& currKeyMatchedFlag,
                      std::vector<int8_t>& referKeyMatchedFlag);

#endif /* SeacrhByEpipolarHandle_hpp */
