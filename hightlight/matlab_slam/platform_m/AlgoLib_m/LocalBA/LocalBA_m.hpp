//
//  LocalBA.hpp
//  Matlab SLAM
//
//  Created by zhaiq on 7/18/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#pragma once
#include <mex.h>

/**
 *@brief Optimize pose and map point by local bandle adjustment.
 * @param prhs[0]
 * @param prhs[1]
 * @param prhs[2]
 * @param prhs[3]
 * @param plhs[0] [output]
 * @param plhs[1] [output]
 */
void mexFunction(int nlhs, mxArray* plhs[],
				 const int nrhs, const mxArray* prhs[]);