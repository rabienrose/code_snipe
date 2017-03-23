//
//  ExtractORB.cpp
//
//  Created by zhaiq on 6/30/16.
//  Copyright (@) 2016 zhaiq. All rights reserved.
//

#include "ExtractORB_m.hpp"
#include <stdio.h>
#include <cassert>
#include <opencvmex.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ORBextractor.h"
#include "typeDef.hpp"

#define RETURN_BY_DEFAULT(nlhs, plhs) \
                        for(int i=0; i<nlhs; i++) { \
                            plhs[i] = mxCreateDoubleScalar(-1); \
                        }

struct KeyWithDesc {
    cv::KeyPoint    key;
    cv::Mat         desc;
};

static inline bool cmp(const KeyWithDesc& key1, const KeyWithDesc& key2)
{
    return key1.key.response > key2.key.response;
}

static ORB_SLAM2::ORBextractor* pExtractor = nullptr;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{    
    assert(nrhs == 1 || nrhs == 2);

    const char* command = "";
    if(nrhs == 2)   command = mxArrayToString(prhs[0]); //init
    
    MESSAGE_PRINT("command : %s\n", command);
    
    if(!strcmp("init", command)) {
        assert(nlhs <= 1); //check output parameters during initiallization.

        //parse parameters to construct ORB extractor.
        const int orb_param_size = 5;
        assert(mxGetN(prhs[1]) == orb_param_size);
        
        double* val = mxGetPr(prhs[1]);
        assert(val);

        int nfeatures       = static_cast<int>(val[0]);
        float scaleFactor   = static_cast<float>(val[1]);
        int nlevels         = static_cast<int>(val[2]);
        int iniFASTTh       = static_cast<int>(val[3]);
        int minFASTth       = static_cast<int>(val[4]);
                
        MESSAGE_PRINT("feature num : %d\n", nfeatures);
        MESSAGE_PRINT("scale factor : %f\n", scaleFactor);
        MESSAGE_PRINT("levels number : %d\n", nlevels);
        MESSAGE_PRINT("iniFASTTh : %d\n", iniFASTTh);
        MESSAGE_PRINT("minFASTth : %d\n", minFASTth);

        if(pExtractor) {
            delete pExtractor;
            pExtractor = nullptr;
        }

        pExtractor = new ORB_SLAM2::ORBextractor(nfeatures, scaleFactor, nlevels, iniFASTTh, minFASTth);
        
        if(nlhs == 1) {
            plhs[0] = pExtractor ? mxCreateDoubleScalar(0) : mxCreateDoubleScalar(-1) ;
        }
        MESSAGE_PRINT("initialization %s\n", (pExtractor ? "success" : "failed"));
        
        return;
    }
    else {
        assert(pExtractor);//check worker handle.
        if(!pExtractor) {
            ERROR_LOG("ORB extractor is not initialized\n");

            return;
        }
        assert((nlhs == 3 || nlhs == 1)); //check output paramaters.

        //read frame.
        cv::Mat frame;
        ocvMxArrayToMat_uint8(prhs[0], frame);

        //compute keys and descriptors.
        std::vector<cv::KeyPoint> keys;
        cv::Mat descriptors;
        if(!frame.empty()) {
            cv::Mat gray = frame;
            if(frame.channels() == 3)
                cv::cvtColor(frame, gray, CV_BGR2GRAY);
            else if(frame.channels() == 4)
                cv::cvtColor(frame, gray, CV_BGRA2GRAY);
            
            try {
                pExtractor->computeKeysAndDescriptor(gray, keys, descriptors);
            } catch(...) {
                ERROR_LOG("Caught a general exception in ExtractORB.");
            }

        }

        if(keys.empty()) {
            RETURN_BY_DEFAULT(nlhs, plhs);
            return;
        }
        assert(keys.size() == descriptors.rows);

        std::vector<KeyWithDesc> allKeys;
        for(int i=0; i<keys.size(); i++) {
            KeyWithDesc single_key;
            single_key.key  = keys[i];
            single_key.desc = descriptors.row(i).clone();

            allKeys.push_back(single_key);
        }

        //std::cout << allKeys.size() << std::endl;

        std::sort(allKeys.begin(), allKeys.end(), cmp);
        allKeys.resize(allKeys.size() / 2);

        // for(int i=0; i<5; i++) {
        //     std::cout << allKeys[i].key.response << " ";
        // }
        // std::cout << std::endl;
        // for(int i=0; i<5; i++) {
        //     std::cout << allKeys[allKeys.size()-1-i].key.response << " ";
        // }
        // std::cout << std::endl << std::endl;

        std::vector<cv::KeyPoint> new_keys;
        cv::Mat new_descriptor = cv::Mat(allKeys.size(), 32, CV_8UC1);
        for(int i=0; i<allKeys.size(); i++) {
            new_keys.push_back(allKeys[i].key);
            allKeys[i].desc.copyTo(new_descriptor.row(i));
        }

        keys = new_keys;
        descriptors = new_descriptor.clone();
        //std::cout << new_keys.size() << "   " << new_descriptor.rows << std::endl;

        bool passErrorType = (nlhs == 3);
        if(passErrorType) plhs[0] = mxCreateDoubleScalar(0);        
        const int rcId = passErrorType ? 1 : 0;
        plhs[rcId] = ocvKeyPointsToStruct(keys);
        
        //
        const char* fieldname = "Octave";
        mxAddField(plhs[rcId], fieldname);

        cv::Mat octaves(keys.size(), 1, CV_32SC1);
        for(int i=0; i<keys.size(); i++) {
            octaves.at<int32_t>(i, 0) = keys[i].octave;
        }
        mxSetFieldByNumber(plhs[rcId], 0, mxGetFieldNumber(plhs[rcId], fieldname), ocvMxArrayFromMat_int32(octaves));


        plhs[rcId + 1] = ocvMxArrayFromMat_uint8(descriptors);
    }
}
