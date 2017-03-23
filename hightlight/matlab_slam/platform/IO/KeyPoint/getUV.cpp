#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::KeyPoint* pKeyPoint = getKeyPointPtr(prhs[0]);
    
    float u = pKeyPoint->m_key.pt.x;
    float v = pKeyPoint->m_key.pt.y;
    
    plhs[0] = mxCreateNumericMatrix(2, 1, mxDOUBLE_CLASS, mxREAL);
    double* ptrr0 = static_cast<double*>(mxGetData(plhs[0]));
    ptrr0[0] = u-1;
    ptrr0[1] = v-1;
}
