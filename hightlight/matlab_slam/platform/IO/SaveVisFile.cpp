#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    std::string path = getMexString(prhs[1]);
    pPlatform->SaveVisFile(path);
}