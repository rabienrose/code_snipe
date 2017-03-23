#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    
    long frameId = getLong(prhs[1])-1;    
    std::string imagePath = getMexString(prhs[2]);
    
    bool re = pPlatform->setFrame(frameId, imagePath.c_str());
}