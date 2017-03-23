
#include <opencvmex.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    long frameId = getLong(prhs[1])-1;
    bool re = pPlatform->tryInitPose(frameId);
    plhs[0] = mxCreateLogicalScalar(re);
    
}
