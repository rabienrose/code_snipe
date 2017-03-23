#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	assert(nlhs==1);
    ygomi::Platform* pPlatform = ygomi::Platform::createInstance();

    plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    setSLAMHandle(plhs[0], pPlatform);
}