#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "MapPoint.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::MapPoint* pMP = getMapPointPtr(prhs[0]);
    long id = pMP->getId()+1;
    plhs[0] = setLong2Double(id);
}