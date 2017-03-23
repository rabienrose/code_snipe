/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SaveToGlobalMap.hpp
 * @brief  Save Matlab data structure to C.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.02        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <matrix.h>
/**
 *@brief Read global map from Matlab.
 *@param src        Source global map.
 *@param mapHandle  Global Map handle.
 */
void readGlobalMapFromM(const mxArray* src, long mapHandle);