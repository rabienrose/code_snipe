
#include <opencvmex.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    
    long channel = getLong(prhs[1]);
    long mpId = getLong(prhs[2])-1;
    
    cv::Mat posiM;
    ocvMxArrayToImage_double(prhs[3], posiM);
    posiM.convertTo(posiM, CV_32FC1);
    
    cv::Mat colorM;
    ocvMxArrayToImage_double(prhs[4], colorM);
    colorM.convertTo(colorM, CV_32FC1);
    
    pPlatform->AddActMpToVisPlat(channel, mpId, posiM, colorM);
}
