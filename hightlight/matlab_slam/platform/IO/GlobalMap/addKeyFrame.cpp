#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "GlobalMap.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::GlobalMap* pGlobalMap = getGlobalMapPtr(prhs[0]);
    long fId = getLong(prhs[1])-1;
    pGlobalMap->addKeyFrame(fId);
}