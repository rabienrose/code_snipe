/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SaveToGlobalMap.hpp
 * @brief  Save C data structure to Matlab.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.04        Zili.Wang     Create
 *******************************************************************************
 */

#pragma once

#include <matrix.h>
/**
 *@brief Save global map to Matlab.
 *@param tar        output mxArray, the memmory should be released by the caller.
 *@param mapHandle  Global Map handle.
 */
void saveGlobalMapToM(mxArray** tar, long mapHandle);