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
    
    const std::vector<ygomi::MapPoint*>& mpList = pGlobalMap->getAllMapPoints();
    std::vector<void*> values;
    values.resize(mpList.size());
    for (int i=0;i<mpList.size();i++){
        values[i] = (void*)mpList[i];
    }
    plhs[0] = setPtr2Uint64List(values);
}