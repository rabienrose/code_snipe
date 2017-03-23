//
//  CalcDistance.cpp
//  ORB_SLAM2
//
//  Created by dzj on 16/7/22.
//
//
#include "mex.h"
#include "opencvmex.hpp"
#include "opencv2/opencv.hpp"
#include "typeDef.hpp"

int calcDescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    
    int dist=0;
    
    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    
    return dist;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs != 2 || nlhs != 1)
    {
        ERROR_LOG("Invalid input or output.\n");
        return;
    }
    
    //read frame.
    cv::Mat frame1;
    ocvMxArrayToMat_uint8(prhs[0], frame1);
    
    cv::Mat frame2;
    ocvMxArrayToMat_uint8(prhs[1], frame2);
    
    int dist = calcDescriptorDistance(frame1, frame2);
    plhs[0] = mxCreateDoubleScalar((double)dist);
}