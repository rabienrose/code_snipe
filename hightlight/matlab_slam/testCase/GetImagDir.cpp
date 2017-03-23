#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::TestCaseParser* pParser = getTestCasePPtr(prhs[0]);
    std::string curCaseName = getMexString(prhs[1]);
    long frameId = getLong(prhs[2]);
    std::string str = pParser->GetImagDir(curCaseName, frameId);
    plhs[0] = mxCreateString(str.c_str());
}