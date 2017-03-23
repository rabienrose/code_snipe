#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::KeyPoint* pKeyPoint = getKeyPointPtr(prhs[0]);
    
    int mplistLen = pKeyPoint->m_mapPointId.size();
    cv::Mat mpList(1, mplistLen, CV_64FC1);
    for (int i=0;i<mplistLen; i++){
        mpList.at<double>(0, i) = pKeyPoint->m_mapPointId[i]+1;
    }
    plhs[0] = ocvMxArrayFromMat_double(mpList);
}