#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    cv::Mat pose = pFrame->getPose().clone();
    plhs[0] = ocvMxArrayFromMat_double(pose);
}