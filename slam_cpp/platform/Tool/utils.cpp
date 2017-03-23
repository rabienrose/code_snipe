/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   utils.cpp
 * @brief  Some utilities functions
 *
 * Change Log:
 *      Date              Who            What
 *      2016.10.09        Qiang.Zhai     Create
 *******************************************************************************
 */

#include "utils.hpp"

bool operator == (const cv::KeyPoint& key1, const cv::KeyPoint& key2)
{
	bool ok = true;
	ok &= (key1.pt.x == key2.pt.x);
	ok &= (key1.pt.y == key2.pt.y);
	ok &= (key1.octave == key2.octave);
	ok &= (key1.angle == key2.angle);
	ok &= (key1.class_id == key2.class_id);
	ok &= (key1.response == key2.response);
	ok &= (key1.size == key2.size);
	
	return ok;
}

bool operator == (const ygomi::KeyPoint& key1, const ygomi::KeyPoint& key2)
{
	bool ok = true;
	ok &= (key1.m_key == key2.m_key);
	
	ok &= (key1.m_descType == key2.m_descType);
	
	const auto& mpIds1 = key1.m_mapPointId;
	const auto& mpIds2 = key2.m_mapPointId;
	ok &= mpIds1.size() == mpIds2.size();
	for(size_t i=0; i<mpIds1.size(); i++) {
		ok &= (mpIds1[i] == mpIds2[i]);
	}
	
	
	if(key1.m_descType == ygomi::FLOAT_DESC) {
		ok &= operator == <float>(key1.m_descriptor, key2.m_descriptor);
	}
	else if(key1.m_descType == ygomi::BINARY_DESC) {
		ok &= operator == <int>(key1.m_descriptor, key2.m_descriptor);
	}
	
	return ok;
}
