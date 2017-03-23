#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    std::string confDir = getMexString(prhs[0]);
    std::string imagRoot = getMexString(prhs[1]);
    ygomi::TestCaseParser* testCase = new ygomi::TestCaseParser(confDir, imagRoot);
    plhs[0] =setPtr2Uint64((void*) testCase);
}