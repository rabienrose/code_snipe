#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "Platform.hpp"
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    assert(nlhs==0 && nrhs== 4);
    ygomi::Platform* pPlatform = getSLAMHandle(prhs[0]);
    if(!pPlatform) {
        std::cout << "failed to parse SLAM Handle\n";
    }


    double* fx = static_cast<double*>(mxGetData(prhs[1]));
    double* fy = static_cast<double*>(mxGetData(prhs[2]));
    double* cx = static_cast<double*>(mxGetData(prhs[3]));
    double* cy = static_cast<double*>(mxGetData(prhs[4]));

//    std::cout << "Set cameraParam\n";
//    std::cout << *fx << std::endl;
//    std::cout << *fy << std::endl;
//    std::cout << *cx << std::endl;
//    std::cout << *cy << std::endl;

    pPlatform->changeCameraParam(*fx, *fy, *cx, *cy);
}