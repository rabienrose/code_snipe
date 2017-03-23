#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    long kpId = getLong(prhs[1])-1;
    pFrame->resetMatchingPair(kpId);
}