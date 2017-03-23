/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LocalMap.hpp
 * @brief  LocalMap is important duing Tracking, it is updated in the co-visibility
 *         graph.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <stdio.h>
#include <unordered_map>
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "TypeDef.hpp"

namespace ygomi
{
    struct LocalMap
    {
        LocalMap()
        {
            m_localKeyFrames.clear();
            m_localMapPoints.clear();
        }
        
        std::unordered_map<long, ygomi::Frame>    m_localKeyFrames;
        std::unordered_map<long, ygomi::MapPoint> m_localMapPoints;
    };
}