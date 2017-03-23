#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::TestCaseParser* pParser = getTestCasePPtr(prhs[0]);
    std::vector<std::string> nameListStr = pParser->GetCaseNames();
    plhs[0] = setStringList(nameListStr);
}