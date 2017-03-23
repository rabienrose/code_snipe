//
//  TriangulateHandle.hpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/5/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#ifndef TriangulateHandle_hpp
#define TriangulateHandle_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

void triangulate(const std::vector<cv::KeyPoint>& currKeys, const cv::Mat& currPose,
                 const std::vector<cv::KeyPoint>& referKeys, const cv::Mat& referPose,
                 const cv::Mat& K,
                 const std::vector<float>& scaleFactorSet,
                 float scaleFactor,
                 std::vector<cv::Mat>& mapPoints,
                 std::vector<int8_t>& status);

#endif /* TriangulateHandle_hpp */
