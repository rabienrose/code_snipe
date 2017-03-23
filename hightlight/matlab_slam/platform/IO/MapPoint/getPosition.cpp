#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "MapPoint.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::MapPoint* pMP = getMapPointPtr(prhs[0]);
    
    const cv::Point3f& posi = pMP->getPosition();
    
    plhs[0] = mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL);
    double* ptrr0 = static_cast<double*>(mxGetData(plhs[0]));
    ptrr0[0] = posi.x;
    ptrr0[1] = posi.y;
    ptrr0[2] = posi.z;
}