#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::TestCaseParser* pParser = getTestCasePPtr(prhs[0]);
    std::string curCaseName = getMexString(prhs[1]);
    int startF = pParser->GetStartFrame(curCaseName);
    int endF = pParser->GetEndFrame(curCaseName);
    plhs[0] = setLong2Double(startF);
    plhs[1] = setLong2Double(endF);
}