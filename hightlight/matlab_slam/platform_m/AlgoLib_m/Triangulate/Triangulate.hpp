//
//  Triangulate.hpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/5/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#ifndef Triangulate_hpp
#define Triangulate_hpp

#include <mex.h>


/**
 * Triangluate matching-points from 2D to 3D.
 * @param prhs[0] Matched keypoints in current frame, including x, y, octave, etc.
 * @param prhs[1] Current camera pose, 3 x 4 double matrix.
 * @param prhs[2] Matched keypoints in reference frame, the same format with prhs[0].
 * @param prhs[3] Reference camera pose, the same with prhs[1].
 * @param prhs[4] Camera intrinsic parameters, 3 x 3 double matrix.
 * @param prhs[5] Scale factors set.
 * @param prhs[6] Scale factor scalar.
 * @param plhs[0] [output] Generated map points.
 * @param plhs[1] [output] Flags to identify whether input keypoints has been triangulated successful.
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]);

#endif /* Triangulate_hpp */
