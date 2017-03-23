/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   OptimizeByLocalBA.hpp
 * @brief  Interface for optimization using local Bandle Adjustment.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.08        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <unordered_map>
#include <vector>
#include "Frame.hpp"
#include "MapPoint.hpp"

void optimizeByLocalBA(std::unordered_map<long, ygomi::Frame*>& keyFrames,
                       std::unordered_map<long, ygomi::MapPoint*>& mapPoints,
                       const cv::Mat& tK,
                       const std::vector<float>& invScaleFactorSigma2,
                       int nIterationNum = 20);