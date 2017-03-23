#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::KeyPoint* pKeyPoint = getKeyPointPtr(prhs[0]);
    
    cv::Mat desc = pKeyPoint->m_descriptor.clone();
    
    plhs[0] = ocvMxArrayFromMat_uint8(desc);
}