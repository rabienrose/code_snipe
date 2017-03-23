//
//  SearchByEpipolar.cpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/4/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "SearchByEpipolar.hpp"
#include <opencvmex.hpp>
#include <cassert>
#include "SeacrhByEpipolarHandle.hpp"
#include "typeDef.hpp"

#ifdef DEFAULT_OUTPUT
#undef DEFAULT_OUTPUT
#endif
#define DEFAULT_OUTPUT(n, y) \
                    for(int i=0; i<n; i++) { \
                        y[i] = mxCreateDoubleMatrix(1, 1, mxREAL);\
                    }

static bool loadFrameInfo(const mxArray* keys, const mxArray* descriptors, const mxArray* pose, FrameInfo* info/*, const mxArray* medianDepth = nullptr*/)
{
    //check input/output parameters.
    assert(keys && descriptors && pose && info);

    //convert matlab structure to OpenCV keys.
    ocvStructToKeyPoints(keys, info->keys);
    if(info->keys.empty())
        return false;
    
    //convert Matlab descriptor to OpenCV format.
    bool bCopy = false;
    cv::Ptr<cv::Mat> img = ocvMxArrayToImage_uint8(descriptors, bCopy);
    info->descriptors = (*img).clone();
    info->descriptors = info->descriptors.t();

    if(info->keys.size() != info->descriptors.rows)
        return false;

    ocvMxArrayToImage_double(pose, info->pose);
    info->pose.convertTo(info->pose, CV_32FC1);

    if(info->pose.rows != 3 || info->pose.cols != 4)
        return false;

    // if(!medianDepth) {
    //     cv::Mat depth;
    //     ocvMxArrayToImage_double(medianDepth, depth);
    //     info->mediaDepth = depth.at<double>(0, 0);
    // }

    return true;
}

static inline bool checkInput(int inputParam, int outputParam)
{
    return inputParam == 8 && outputParam == 3;
}

void mexFunction(int outC, mxArray* outP[],
                 int inC, const mxArray* inP[])
{
    assert(checkInput(inC, outC));

    //load current un-matched frame info
    FrameInfo currUnmatched;
    if(!loadFrameInfo(inP[0], inP[1], inP[2], &currUnmatched)) {
        ERROR_LOG("invalid input in mex during searchByEpipolar1.\n");
        DEFAULT_OUTPUT(outC, outP);
        return;
    }
        
    //load selected un-matched frame info
    FrameInfo referUnmatched;
    if(!loadFrameInfo(inP[3], inP[4], inP[5], &referUnmatched)) {
        ERROR_LOG("invalid input in mex during searchByEpipolar2.\n");
        DEFAULT_OUTPUT(outC, outP);
        return;
    }
    
    //load camera intrinsic parameters.
    cv::Mat K;
    ocvMxArrayToImage_double(inP[6], K);
    K.convertTo(K, CV_32FC1);
    if(K.rows != 3 || K.cols != 3) {
        ERROR_LOG("invalid input in mex during searchByEpipolar3.\n");
        DEFAULT_OUTPUT(outC, outP);
        return;
    }

    //load scale factors.
    std::vector<float> scaleFactors;
    const size_t N = mxGetN(inP[7]);
    scaleFactors.reserve(N);
    
    double* val = static_cast<double*>(mxGetData(inP[7]));
    assert(val);
    for(int i=0; i<N; i++)
        scaleFactors.push_back(val[i]);
    
    std::vector<std::pair<int, int> > matchedPairs;
    std::vector<int8_t> currKeyMatchedFlag;
    std::vector<int8_t> referKeyMatchedFlag;
    try {
        searchByEpipolar(currUnmatched, referUnmatched, K, scaleFactors, matchedPairs, currKeyMatchedFlag, referKeyMatchedFlag);
    } catch(...) {
        ERROR_LOG("Caught a general exception in searchByEpipolar.");
    }
    
    ///write back.
    //matched pairs
    cv::Mat cvMatchingPairs(matchedPairs.size(), 2, CV_32SC1);
    int* ptr = nullptr;
    for(int i=0; i<matchedPairs.size(); i++) {
        ptr = cvMatchingPairs.ptr<int>(i);
        *ptr++ = matchedPairs[i].first + 1; //in matlab, index count from 1.
        *ptr = matchedPairs[i].second + 1;
    }
    outP[0] = ocvMxArrayFromImage_int32(cvMatchingPairs);
    
    //current keys matched flag
    cv::Mat currKeyFlag(currKeyMatchedFlag.size(), 1, CV_8UC1);
    memcpy(currKeyFlag.data, &currKeyMatchedFlag[0], currKeyMatchedFlag.size() * sizeof(uint8_t));
    outP[1] = ocvMxArrayFromImage_uint8(currKeyFlag);
    
    //reference keys matched flag
    cv::Mat referKeyFlag(referKeyMatchedFlag.size(), 1, CV_8UC1);
    memcpy(referKeyFlag.data, &referKeyMatchedFlag[0], referKeyMatchedFlag.size() * sizeof(uint8_t));
    outP[2] = ocvMxArrayFromImage_uint8(referKeyFlag);
}