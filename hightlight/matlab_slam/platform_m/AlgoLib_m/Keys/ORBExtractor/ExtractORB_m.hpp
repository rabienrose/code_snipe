//
//  ExtractORB.hpp
//
//  Created by zhaiq on 6/30/16.
//  Copyright (@) 2016 zhaiq. All rights reserved.
//

#ifndef ExtractORB_hpp
#define ExtractORB_hpp

#include <mex.h>

/**
 *@param prhs[0] Working mode. 'init', construct and initialize ORB extractor.
 *  @param prhs[1] Parameters to construct ORB extractor.
 *                 sort by max number of features, scalefactor, level num, initial fast threshold, min fast threshold.
 *  @param plhs[0] [output] Initialized flag, return 0 if success, <0 otherwise.
 *
 *@param prhs[0] Other working mode. after initialize successful, detect keys and compute ORB descriptors.
 *  @param prhs[0] Input image, ONLY gray-scale is allowable.
 *  @param plhs[0] [output] Processed flag, return 0 if success, <0 otherwise.
 *  @param plhs[1] [output] Detected keys and descriptors.
 *							each element is defined as struct{"KeyPoint", "Descriptor"}
 *							each KeyPoint is order as point_x, point_y, angle, response, size, class_id.
 *							each Descriptor is a 1 x N dimension array, for ORB, N = 8.
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]);

#endif /* ExtractORB_hpp */