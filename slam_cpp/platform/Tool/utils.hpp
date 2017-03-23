/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utils.hpp
 * @brief  
 *
 * Change Log:
 *      Date              Who            What
 *      2016.10.09        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "TypeDef.hpp"

template<typename T>
bool operator == (const cv::Mat& m1, const cv::Mat& m2)
{
	bool ok = true;
	ok &= (m1.cols == m2.cols && m1.rows == m2.rows && m1.type() == m2.type());
	for(int j=0; j<m1.rows; j++) {
		const T* ptr1 = m1.ptr<T>(j);
		const T* ptr2 = m2.ptr<T>(j);
		for(int i=0; i<m1.cols; i++) {
			ok &= (*ptr1++ == *ptr2++);
		}
	}
	
	return ok;
}

bool operator == (const cv::KeyPoint& key1, const cv::KeyPoint& key2);

bool operator == (const ygomi::KeyPoint& key1, const ygomi::KeyPoint& key2);
