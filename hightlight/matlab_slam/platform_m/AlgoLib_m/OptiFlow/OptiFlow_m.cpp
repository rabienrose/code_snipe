#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <cassert>
#include "optflowIni.h"
#include "OptiFlow_m.h"
#include "opencvmex.hpp"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    cv::Mat frame1;
    cv::Mat frame2;
    ocvMxArrayToImage_uint8(prhs[0], frame1);
    ocvMxArrayToImage_uint8(prhs[1], frame2);
    cv::vector<cv::KeyPoint> kpts1, kpts2;
    ocvStructToKeyPoints(prhs[2],kpts1);
    ocvStructToKeyPoints(prhs[3],kpts2);
    
    cv::Mat desc1, desc2;
    ocvMxArrayToMat_uint8(prhs[4], desc1);
    ocvMxArrayToMat_uint8(prhs[5], desc2);
    cv::Mat mK;
    ocvMxArrayToMat_double(prhs[6], mK);
    mK.convertTo(mK, CV_32FC1);
    
    algo::OptflowIni optflowIni_;
    optflowIni_.init(mK);
    optflowIni_(frame1, kpts1, desc1);
    optflowIni_(frame2, kpts2, desc2);
    std::vector<std::pair<std::size_t, std::size_t> > matches;
    std::vector<cv::Mat> x3Ds;
    cv::Mat R,t;

    if(optflowIni_.getIniRet(matches, x3Ds, R, t))
    {
        cv::Mat matchPair(matches.size(),2, CV_32SC1);
        for(int i=0; i<matches.size();i++){
            matchPair.at<int>(i,0) = matches[i].first;
            matchPair.at<int>(i,1) = matches[i].second;
        }
        cv::Mat pose = algo::makeT(R, t);
        pose.convertTo(pose, CV_64FC1);
        plhs[0] =ocvMxArrayFromMat_int32(matchPair);
        plhs[1] =ocvMxArrayFromMat_double(pose);
    }
}