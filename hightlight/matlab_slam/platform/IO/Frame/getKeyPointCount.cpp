#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include "TypeDef.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    int kpCount = pFrame->getKeyPoints().size();
    plhs[0] = setLong2Double(kpCount);
}