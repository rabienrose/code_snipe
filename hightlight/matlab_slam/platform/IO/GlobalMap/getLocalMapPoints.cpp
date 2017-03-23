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
    
    long kfId = getLong(prhs[1])-1;
    
    std::vector<ygomi::MapPoint*> plocalMapPoints;
    pGlobalMap->getLocalMapPoints(kfId, plocalMapPoints);
    std::vector<void*> values;
    values.resize(plocalMapPoints.size());
    for (int i=0;i<plocalMapPoints.size();i++){
        values[i] = (void*)plocalMapPoints[i];
    }
    plhs[0] = setPtr2Uint64List(values);
    
}