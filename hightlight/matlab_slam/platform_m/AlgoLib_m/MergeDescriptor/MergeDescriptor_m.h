#ifndef MergeDesc_hpp
#define MergeDesc_hpp
#include <mex.h>

/**
 *  @param prhs[0] descriptors for each kp, each col is one descriptor
 *  @param plhs[0] merged descriptor, each col is one descriptor
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]);

#endif /* MergeDesc_hpp */