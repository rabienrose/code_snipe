
#include <opencvmex.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    
    std::string path = getMexString(prhs[1]);
    
    long channel = getLong(prhs[2]);
    long dataType = getLong(prhs[3]);
    
    pPlatform->SaveVisData(path, channel, dataType);
}
