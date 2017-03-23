#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    std::string name = getMexString(prhs[1]);
    pFrame->setName(name);
}