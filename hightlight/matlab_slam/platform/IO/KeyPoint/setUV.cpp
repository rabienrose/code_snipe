#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::KeyPoint* pKeyPoint = getKeyPointPtr(prhs[0]);
    
    double* ptr1 = static_cast<double*>(mxGetData(prhs[1]));
    double* ptr2 = static_cast<double*>(mxGetData(prhs[2]));
    
    pKeyPoint->m_key.pt.x = *ptr1-1;
    pKeyPoint->m_key.pt.y = *ptr2-1;
}