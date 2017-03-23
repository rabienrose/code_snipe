#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform::releaseInstance();
}