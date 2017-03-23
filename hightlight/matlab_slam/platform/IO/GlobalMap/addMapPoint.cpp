#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::GlobalMap* pGlobalMap = getGlobalMapPtr(prhs[0]);
    ygomi::MapPoint mp;
    pGlobalMap->addMapPoint(mp);
    ygomi::MapPoint* pmp = pGlobalMap->getMapPoint(mp.getId());
    plhs[0] =setPtr2Uint64((void*) pmp);
}