#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::TestCaseParser* pParser = getTestCasePPtr(prhs[0]);
    std::string curCaseName = getMexString(prhs[1]);
    std::string str = pParser->GetConfigDir(curCaseName);
    plhs[0] = mxCreateString(str.c_str());
}