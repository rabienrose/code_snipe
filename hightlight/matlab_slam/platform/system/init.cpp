#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	assert(nlhs==0 && nrhs==1);
	ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    if(!pPlatform) {
    	std::cout << "failed to parse SLAM Handle\n";
    }
    std::string tStr = getMexString(prhs[1]);
    pPlatform->init(tStr.c_str());
}