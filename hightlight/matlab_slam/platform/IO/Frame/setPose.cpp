#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Frame.hpp"
#include "SLAMHandeUtil.h"
#include <opencv2/opencv.hpp>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Frame* pFrame = getFramePtr(prhs[0]);
    cv::Mat pose;
    ocvMxArrayToImage_double(prhs[1], pose);
    pFrame->setPose(pose.clone());
}