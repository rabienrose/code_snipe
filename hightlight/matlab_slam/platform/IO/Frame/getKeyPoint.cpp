#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);   
    long kpId = getLong(prhs[1]);
    kpId = kpId-1;
    const ygomi::KeyPoint* pKp = &pFrame->getKeyPoint(kpId);
    plhs[0] = setPtr2Uint64((void *)pKp);
}