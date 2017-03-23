#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::TestCaseParser* pParser = getTestCasePPtr(prhs[0]);
    std::string curCaseName = getMexString(prhs[1]);
    float fx, fy, cx, cy;
    pParser->GetCamConfig(curCaseName, fx, fy, cx, cy);
    plhs[0] = setFloat2Double(fx);
    plhs[1] = setFloat2Double(fy);
    plhs[2] = setFloat2Double(cx);
    plhs[3] = setFloat2Double(cy);
}