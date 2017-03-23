#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::KeyPoint* pKeyPoint = getKeyPointPtr(prhs[0]);
    
    cv::Mat mpList;
    ocvMxArrayToImage_double(prhs[1], mpList);
    int mplistLen = mpList.cols;
    for (int i=0;i<mplistLen; i++){
        pKeyPoint->m_mapPointId.push_back(mpList.at<double>(0, i)-1);
    }
}