//
//  SearchByEpipolar.hpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/4/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#ifndef SearchByEpipolar_hpp
#define SearchByEpipolar_hpp

#include <mex.h>

/**
 * Search matching pairs by epipolar, before triangulation un-matched keys, 
 * matching pairs should be obtained by this.
 * @param prhs[0] Keypoints in current frame.
 * @param prhs[1] Descriptors of keypoints in current frame.
 * @param prhs[2] Current camera pose.
 * @param prhs[3] Keypoints in selected frame.
 * @param prhs[4] Descriptors of keypoints in selected frame.
 * @param prhs[5] Selected frame camera pose.
 * @param prhs[6] Camera intrinsic parameters.
 * @param prhs[7] Scale factors set.
 * @param plhs[0] [output] Searched matching pairs.
 * @param plhs[1] [output] Matching flag for current keys.
 * @param plhs[2] [output] Matching flag for selected frame.
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]);


#endif /* SearchByEpipolar_hpp */
