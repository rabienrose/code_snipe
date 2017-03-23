//
//  SeacrhByEpipolarHandle.cpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/4/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "SeacrhByEpipolarHandle.hpp"
#include "typeDef.hpp"

inline static cv::Mat getRotation(const cv::Mat& pose)
{
    return pose.rowRange(0,3).colRange(0,3).clone();;
}

inline static cv::Mat getTranslation(const cv::Mat& pose)
{
    return pose.rowRange(0,3).col(3).clone();
}

inline static cv::Mat computeCameraCenter(const cv::Mat& pose)
{
    return -getRotation(pose).t() * getTranslation(pose);
}

inline static cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) << 0,               -v.at<float>(2),   v.at<float>(1),
                                    v.at<float>(2),               0,    -v.at<float>(0),
                                    -v.at<float>(1),  v.at<float>(0),              0);
}

inline static cv::Mat ComputeF12(const cv::Mat& pose1, const cv::Mat& pose2, const cv::Mat& K)
{
    cv::Mat R1w = getRotation(pose1);
    cv::Mat t1w = getTranslation(pose1);
    cv::Mat R2w = getRotation(pose2);
    cv::Mat t2w = getTranslation(pose2);
    
    cv::Mat R12 = R1w * R2w.t();
    cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;
    
    cv::Mat t12x = SkewSymmetricMatrix(t12);
    
    return K.t().inv() * t12x * R12 * K.inv();
}

static int computeDescriptorDistance(const cv::Mat& a, const cv::Mat& b)
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

bool checkDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& F12, const std::vector<float>& scaleFactorSet)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);
    
    const float num = a*kp2.pt.x+b*kp2.pt.y+c;
    
    const float den = a*a+b*b;
    
    if(den==0)
        return false;
    
    const float dsqr = num*num/den;
    
    return dsqr < 3.84 * scaleFactorSet[kp2.octave] * scaleFactorSet[kp2.octave];
}

static void searchForTriangulation(const FrameInfo& currUnmatched,
                                   const FrameInfo& referUnmatched,
                                   const cv::Mat& K,
                                   const cv::Mat& Fundamental,
                                   const std::vector<float>& scaleFactorSet,
                                   std::vector<std::pair<int, int> >& matchedPairs,
                                   std::vector<int8_t>& currKeyMatchedFlag,
                                   std::vector<int8_t>& referKeyMatchedFlag)
{
    MESSAGE_PRINT("search for triangulation: step 1.\n");

    //Compute epipole in second image
    cv::Mat Ow1 = computeCameraCenter(currUnmatched.pose);
    cv::Mat R2w = getRotation(referUnmatched.pose);
    cv::Mat t2w = getTranslation(referUnmatched.pose);
    cv::Mat C2 = R2w * Ow1 + t2w;
    const float invz = 1.0f/C2.at<float>(2);
    
    float fx = K.at<float>(0, 0);
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    const float ex = fx * C2.at<float>(0) * invz + cx;
    const float ey = fy * C2.at<float>(1) * invz + cy;
    
    int currKeysCount  = static_cast<int>(currUnmatched.keys.size());
    int referKeysCount = static_cast<int>(referUnmatched.keys.size());

    currKeyMatchedFlag.resize(currKeysCount, 0);//matching info of all current keys.
    referKeyMatchedFlag.resize(referKeysCount, 0);
    std::vector<int> currKeyMatchedIdx(currKeysCount, -1);

    MESSAGE_PRINT("search for triangulation: step 2.\n");
    
    int matchedCount = 0;
    for(int k=0; k<currKeysCount; k++) {//for each current key, search a match.
        const cv::KeyPoint& currKey = currUnmatched.keys[k];
        const cv::Mat& currDesc     = currUnmatched.descriptors.row(k);
        
        // find the nearest descriptor
        const int TH_LOW = 50;
        int bestDist = TH_LOW;
        int bestIdx2 = -1;
        for(int i=0; i<referKeysCount; i++) {           
            // If we have already matched, skip
            if(referKeyMatchedFlag[i] == 1) continue;

            //not pass the epipolar check
            const cv::KeyPoint& referKey = referUnmatched.keys[i];
            if(!checkDistEpipolarLine(currKey, referKey, Fundamental, scaleFactorSet))
                continue;

            const cv::Mat& referDesc = referUnmatched.descriptors.row(i);
            
            int dist = computeDescriptorDistance(currDesc, referDesc);
            if(dist > TH_LOW || dist > bestDist)
                continue;
            
            
            const float distex = ex-referKey.pt.x;
            const float distey = ey-referKey.pt.y;
            if(distex*distex + distey*distey < 100*scaleFactorSet[referKey.octave])
                continue;
            
            //if(checkDistEpipolarLine(currKey, referKey, Fundamental, scaleFactorSet))
            {
                bestIdx2 = i;
                bestDist = dist;
            }
        }
        
        if(bestIdx2 >= 0) {
            currKeyMatchedIdx[k]          = bestIdx2;
            currKeyMatchedFlag[k]         = 1;
            referKeyMatchedFlag[bestIdx2] = 1;
            
            matchedCount++;
        }
    }
    
    MESSAGE_PRINT("search for triangulation: step 3.\n");

    matchedPairs.clear();
    matchedPairs.reserve(matchedCount);
    for (size_t i=0; i<currKeyMatchedFlag.size(); i++) {
        if(currKeyMatchedFlag[i] == 0) {
            assert(currKeyMatchedIdx[i] == -1);
            continue;
        }

        matchedPairs.push_back(std::make_pair(i, currKeyMatchedIdx[i]));
    }
    MESSAGE_PRINT("search for triangulation: success.\n");
    MESSAGE_PRINT("detail info: \n \
                   current unmatched keys: %d,\n\
                   reference unmatched keys: %d,\n\
                   find matchies: %d\n", currUnmatched.keys.size(), referUnmatched.keys.size(), matchedCount);
}

void searchByEpipolar(const FrameInfo& currUnmatched,
                      const FrameInfo& referUnmacthed,
                      const cv::Mat& K,
                      const std::vector<float>& scaleFactorsSet,
                      std::vector<std::pair<int, int> >& matchedPairs,
                      std::vector<int8_t>& currKeyMatchedFlag,
                      std::vector<int8_t>& referKeyMatchedFlag)
{
    // cv::Mat vBaseline = computeCameraCenter(currUnmatched.pose) - computeCameraCenter(referUnmacthed.pose);
    // const float baseline = cv::norm(vBaseline);
    //
    // const float ratioBaselineDepth = baseline / referUnmacthed.mediaDepth;
    //
    // if(ratioBaselineDepth<0.01) return;

    // Compute Fundamental Matrix
    cv::Mat F12 = ComputeF12(currUnmatched.pose, referUnmacthed.pose, K);
    
    // Search matches that fullfil epipolar constraint
    searchForTriangulation(currUnmatched, referUnmacthed, K, F12, scaleFactorsSet, matchedPairs, currKeyMatchedFlag, referKeyMatchedFlag);
}