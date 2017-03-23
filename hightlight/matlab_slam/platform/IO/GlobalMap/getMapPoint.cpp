#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "GlobalMap.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::GlobalMap* pGlobalMap = getGlobalMapPtr(prhs[0]);
    long mpId = getLong(prhs[1])-1;
    ygomi::MapPoint* mp = pGlobalMap->getMapPoint(mpId);
    plhs[0] =setPtr2Uint64((void*) mp);
}