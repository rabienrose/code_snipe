#ifndef OptiFlow_hpp
#define OptiFlow_hpp

#include <mex.h>

/**
 *  @param prhs[0] First Image
 *  @param prhs[1] Second Image
 *  @param prhs[2] First kps
 *  @param prhs[3] Second kps
 *  @param prhs[4] First desc
 *  @param prhs[5] Second desc
 *  @param plhs[0] [output] Initialized flag, return 0 if success, <0 otherwise.
 *  @param prhs[0] Input image path.
 *  @param plhs[0] [output] Processed flag, return 0 if success, >0 or <0 otherwise.
 *  @param plhs[1] [output] Detected keys and descriptors.
 *							each element is defined as struct{"KeyPoint", "Descriptor"}
 *							each KeyPoint is order as point_x, point_y, angle, response, size, class_id.
 *							each Descriptor is a 1 x N dimension array, for ORB, N = 8.
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]);

#endif /* OptiFlow_hpp */