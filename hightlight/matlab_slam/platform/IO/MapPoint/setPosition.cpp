#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "MapPoint.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::MapPoint* pMP = getMapPointPtr(prhs[0]);
    
    cv::Mat posiM;
    ocvMxArrayToImage_double(prhs[1], posiM);
    
    cv::Point3f posi;
    posi.x = posiM.at<double>(0, 0);
    posi.y = posiM.at<double>(0, 1);
    posi.z = posiM.at<double>(0, 2);
    
    pMP->setPosition(posi);
}