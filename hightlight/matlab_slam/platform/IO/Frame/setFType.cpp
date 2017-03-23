#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    long type = getLong(prhs[1]);
    pFrame->setType(ygomi::FrameType(type));
}