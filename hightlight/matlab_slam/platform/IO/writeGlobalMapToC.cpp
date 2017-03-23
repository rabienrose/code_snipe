
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    pPlatform->getGlobalMapFromMatlab(prhs[1]);
}