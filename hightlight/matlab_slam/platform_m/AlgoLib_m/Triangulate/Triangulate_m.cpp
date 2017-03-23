//
//  Triangulate.cpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/5/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "Triangulate.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cassert>
#include <opencvmex.hpp>
#include "TriangulateHandle.hpp"
#include "typeDef.hpp"

#ifdef DEFAULT_OUTPUT
#undef DEFAULT_OUTPUT
#endif
#define DEFAULT_OUTPUT(n, y) \
                    for(int i=0; i<n; i++) { \
                        y[i] = mxCreateDoubleMatrix(1, 1, mxREAL);\
                    }

static bool parseParameters(const mxArray* keys, const mxArray* pose, std::vector<cv::KeyPoint>& parsedKeys, cv::Mat& parsedPose)
{
    //check input parameters.
    assert(keys && pose);
    
    //convert Matlab structure to OpenCV keys.
    ocvStructToKeyPoints(keys, parsedKeys);
    if(parsedKeys.empty())
        return false;
    
    //convert Matlab pose to OpenCV format.
    ocvMxArrayToImage_double(pose, parsedPose);
    if(parsedPose.empty() || parsedPose.cols != 4 || parsedPose.rows != 3)
        return false;

    return true;
}

void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    assert(nlhs == 7 && nlhs == 2);

    //load current camera parameters.
    std::vector<cv::KeyPoint> currKeys;
    cv::Mat currPose;
    if(!parseParameters(prhs[0], prhs[1], currKeys, currPose)) {
        DEFAULT_OUTPUT(nlhs, plhs);
        ERROR_LOG("invalid input in mex while triangulation1.\n");
        return;
    }
    
    //load reference camera parameters.
    std::vector<cv::KeyPoint> referKeys;
    cv::Mat referPose;
    if(!parseParameters(prhs[2], prhs[3], referKeys, referPose)) {
        DEFAULT_OUTPUT(nlhs, plhs);
        ERROR_LOG("invalid input in mex while triangulation2.\n");
        return;
    }
    
    //load camera intrinsic parameters.
    cv::Mat K;
    ocvMxArrayToImage_double(prhs[4], K);
    
    //load scale factors.
    const size_t N = mxGetN(prhs[5]);
    std::vector<float> scaleFactorSet;
    scaleFactorSet.reserve(N);
    double* val = nullptr;
    val = static_cast<double*>(mxGetData(prhs[5]));    assert(val);
    for(size_t i=0; i<N; i++)
        scaleFactorSet.push_back(val[i]);
    
    //load single scale.
    val = static_cast<double*>(mxGetData(prhs[6]));    assert(val);
    float scaleFactor = val[0];


    //triangulate
    std::vector<cv::Mat> mapPoints;
    std::vector<int8_t> status;
    try {
        triangulate(currKeys, currPose,
                     referKeys, referPose,
                     K,
                     scaleFactorSet,
                     scaleFactor,
                     mapPoints,
                     status);
    } catch(...) {
        ERROR_LOG("Caught a general exception in Triangulate.");
    }
    
    ///write back
    //triangulated result.
    cv::Mat allMapPoints(static_cast<int>(mapPoints.size()), 3, CV_32FC1);
    for(size_t j=0; j<mapPoints.size(); j++) {
        const cv::Mat& m = mapPoints[j].t();
        m.copyTo(allMapPoints.row(static_cast<int>(j)));
    }
    //allMapPoints = allMapPoints.t();
    plhs[0] = ocvMxArrayFromMat_single(allMapPoints);
    
    //flags to identify which matching-pair triangulate successful.
    int ndim = 2;
    mwSize dims[2] = {static_cast<mwSize>(status.size()), 1};
    plhs[1] = mxCreateNumericArray(ndim, dims, mxLOGICAL_CLASS, mxREAL);
    int8_t* ptr = static_cast<int8_t*>(mxGetData(plhs[1]));
    assert(ptr);
    for(int i=0; i<status.size(); i++)
        *ptr++ = status[i];
 }