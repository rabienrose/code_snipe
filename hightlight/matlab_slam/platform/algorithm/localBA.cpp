
#include <opencvmex.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    long frameId = getLong(prhs[1])-1;
    long keyFrameCount = getLong(prhs[2]);
    pPlatform->localBA(frameId, keyFrameCount);
}
