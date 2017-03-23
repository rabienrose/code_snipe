/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   TypeDef.hpp
 * @brief  Some basic structures are defined here.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

//#define DEBUG_MESSAGE_PRINT

#ifdef DEBUG_MESSAGE_PRINT
#   define MESSAGE_PRINT(...)   mexPrintf(__VA_ARGS__)
#else
#   define MESSAGE_PRINT(...)
#endif

#define toString( name ) #name

namespace ygomi
{
    enum DesciptorType
    {
        BINARY_DESC = 0,
        FLOAT_DESC  = 1
    };
    
    struct KeyPoint
    {
        std::vector<long>    m_mapPointId; // Which map point that key point is corresponding to.
                                           // NOTE!!! in general, there is at most only one map point coorespond to current key,
                                           // actually, multile map points may be projected to the same key point because the estimated error.
                                           // we save the original matching infomation, and optimize this by g2o.
        cv::KeyPoint         m_key;        // Key point position.
        cv::Mat              m_descriptor; // Key point descriptor.
        static DesciptorType m_descType;   // Distance type.
    };

    // Each track item is used to find the relationship bewteen map point and its key point,
    // it gives the information that the key point(2D) is in which frame, and which key.
    // NOTE!!! To obtain better understanding, we abandon memory layout.
    struct Track
    {
        Track() = default;
        Track(long frameId, size_t keyId) : m_frameId(frameId), m_keyPointId(keyId) { }
        
        long   m_frameId;    // Shows that the key point is in which frame, double-check its frame type must be KEY_FRAME.
        size_t m_keyPointId; // Which key point of key frame.        
    };
}