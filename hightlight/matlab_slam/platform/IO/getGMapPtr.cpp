#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "GlobalMap.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    ygomi::GlobalMap *gMapPtr = pPlatform->getGMapPtr();
    plhs[0] =setPtr2Uint64((void*) gMapPtr);
}