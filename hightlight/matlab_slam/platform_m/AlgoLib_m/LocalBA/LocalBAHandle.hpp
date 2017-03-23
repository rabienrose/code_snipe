//
//  LocalBAHandle.hpp
//  Matlab SLAM
//
//  Created by zhaiq on 7/18/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#pragma once

#include <stdio.h>
//#include <map>
#include <unordered_map>
#include <vector>
#include "typeDef.hpp"

void localBAHandle(std::unordered_map<long, ygomi::Frame>& keyFrames,
                   std::unordered_map<long, ygomi::MapPoint>& mapPoints,
                   const cv::Mat& tK,
                   const std::vector<float>& invScaleFactorSigma2);