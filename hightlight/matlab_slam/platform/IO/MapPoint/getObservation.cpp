#include <opencvmex.hpp>
#include <iostream>
#include <string>
#include "MapPoint.hpp"
#include "TypeDef.hpp"
#include <opencv2/opencv.hpp>
#include "SLAMHandeUtil.h"

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    ygomi::MapPoint* pMP = getMapPointPtr(prhs[0]);
    
    const std::vector<ygomi::Track>& tracks = pMP->getObservation();
    cv::Mat trackMat(2, tracks.size(), CV_64FC1);
    for (int i=0;i<tracks.size(); i++){
        trackMat.at<double>(i, 0) = tracks[i].m_frameId+1;
        trackMat.at<double>(i, 1) = tracks[i].m_keyPointId+1;
    }
    
    plhs[0] = ocvMxArrayFromMat_double(trackMat);
}