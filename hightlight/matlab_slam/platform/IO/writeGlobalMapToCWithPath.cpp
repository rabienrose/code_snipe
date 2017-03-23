
#include "Platform.hpp"
#include "SLAMHandeUtil.h"
#include "string.h"
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    if(!pPlatform) {
        std::cout << "failed to parse SLAM Handle\n";
    }


    //read global map from certain path.
    int strLenght = mxGetN(prhs[1]);
    char16_t *ss = mxGetChars(prhs[1]);
    std::string tStr;
    for (int n = 0;n<strLenght;n++){
        tStr.push_back(ss[n]);
    }
//    std::cout << "String"  << pPlatform << std::endl;
//    std::cout << tStr << std::endl;

    pPlatform->getGobalMapFromMatlab(tStr);
}