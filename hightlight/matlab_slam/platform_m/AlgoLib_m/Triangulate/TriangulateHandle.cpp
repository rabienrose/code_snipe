//
//  TriangulateHandle.cpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/5/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "TriangulateHandle.hpp"
#include <assert.h>
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

void triangulate(const std::vector<cv::KeyPoint>& currKeys, const cv::Mat& tempCurrPose,
                 const std::vector<cv::KeyPoint>& referKeys, const cv::Mat& tempReferPose,
                 const cv::Mat& tempK,
                 const std::vector<float>& scaleFactorSet,
                 float scaleFactor,
                 std::vector<cv::Mat>& mapPoints,
                 std::vector<int8_t>& successFlag)
{
    MESSAGE_PRINT("start triangulate\n");

    assert(currKeys.size() == referKeys.size());
    assert(tempCurrPose.type() == CV_64FC1 || tempCurrPose.type() == CV_32FC1);
    assert(tempReferPose.type() == CV_64FC1 || tempReferPose.type() == CV_32FC1);    
    assert(tempK.type() == CV_64FC1 || tempK.type() == CV_32FC1);

    cv::Mat currPose = tempCurrPose;
    if(tempCurrPose.type() == CV_64FC1)     tempCurrPose.convertTo(currPose, CV_32FC1);

    cv::Mat referPose = tempReferPose;
    if(referPose.type() == CV_64FC1)    tempReferPose.convertTo(referPose, CV_32FC1);

    cv::Mat K = tempK;
    if(tempK.type() == CV_64FC1)            tempK.convertTo(K, CV_32FC1);


    mapPoints.clear();    mapPoints.reserve(currKeys.size());
    successFlag.clear();  successFlag.reserve(currKeys.size());
    
    //load camera intrinsic parameters.
    const float &fx = K.at<float>(0, 0);
    const float &fy = K.at<float>(1, 1);
    const float &cx = K.at<float>(0, 2);
    const float &cy = K.at<float>(1, 2);
    const float &invfx = 1.0f / fx;
    const float &invfy = 1.0f / fy;
    
    //convert current camera pose.
    cv::Mat Rcw1 = getRotation(currPose);
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = getTranslation(currPose);
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = computeCameraCenter(currPose);
    
    //convert reference camera pose.
    cv::Mat Rcw2 = getRotation(referPose);
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = getTranslation(referPose);
    cv::Mat Tcw2(3,4,CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0,3));
    tcw2.copyTo(Tcw2.col(3));
    cv::Mat Ow2 = computeCameraCenter(referPose);
    
    const float ratioFactor = 1.5f*scaleFactor;
    
    // Triangulate each match
    const int nmatches = static_cast<int>(currKeys.size());
    for(int ikp=0; ikp<nmatches; ikp++)
    {
        MESSAGE_PRINT("key Id: %d\n", ikp);
        MESSAGE_PRINT("step 1\n");

        const cv::KeyPoint &kp1 = currKeys[ikp];
        const cv::KeyPoint &kp2 = referKeys[ikp];
        
        // Check parallax between rays
        cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx)*invfx, (kp1.pt.y-cy)*invfy, 1.0);
        cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx)*invfx, (kp2.pt.y-cy)*invfy, 1.0);
        
        cv::Mat ray1 = Rwc1*xn1;
        cv::Mat ray2 = Rwc2*xn2;
        const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
        
        float cosParallaxStereo = cosParallaxRays+1;
        
        MESSAGE_PRINT("step 2\n");

        cv::Mat x3D;
        if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0) //check parallex
        {
            // Linear Triangulation Method(DLT)
            cv::Mat A(4,4,CV_32F);
            A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
            A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
            A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
            A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
            
            cv::Mat w,u,vt;
            cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
            
            x3D = vt.row(3).t();
            
            if(x3D.at<float>(3)==0) { //check scale factor
                successFlag.push_back(0);
                MESSAGE_PRINT("scale factor is zero\n");
                continue;
            }
            
            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
            
        }
        else { //No stereo and very low parallax
            successFlag.push_back(0);
            MESSAGE_PRINT("low parallax\n");
            continue;
        }
        
        MESSAGE_PRINT("step 3\n");

        cv::Mat x3Dt = x3D.t();
        
        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
        float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
        if(z1 <= 0 || z2 <= 0) {
            successFlag.push_back(0);
            MESSAGE_PRINT("out of scene\n");
            continue;
        }
        
        //Check reprojection error in first keyframe
        const float &sigmaSquare1 = scaleFactorSet[kp1.octave] * scaleFactorSet[kp1.octave];
        const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
        const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
        const float invz1 = 1.0/z1;
        {
            float u1 = fx*x1*invz1+cx;
            float v1 = fy*y1*invz1+cy;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1) {
                successFlag.push_back(0);
                MESSAGE_PRINT("reprojection error is too large in first key frame\n");
                continue;
            }
        }
        
        //Check reprojection error in second keyframe
        const float sigmaSquare2 = scaleFactorSet[kp2.octave] * scaleFactorSet[kp2.octave];
        const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
        const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
        const float invz2 = 1.0/z2;
        {
            float u2 = fx*x2*invz2+cx;
            float v2 = fy*y2*invz2+cy;
            float errX2 = u2 - kp2.pt.x;
            float errY2 = v2 - kp2.pt.y;
            if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2) {
                successFlag.push_back(0);
                MESSAGE_PRINT("reprojection error is too large in second key frame\n");
                continue;
            }
        }
        
        MESSAGE_PRINT("step 4\n");

        //Check scale consistency
        cv::Mat normal1 = x3D-Ow1;
        float dist1 = cv::norm(normal1);
        
        cv::Mat normal2 = x3D-Ow2;
        float dist2 = cv::norm(normal2);
        
        if(dist1==0 || dist2==0) {
            successFlag.push_back(0);
            MESSAGE_PRINT("distance is none\n");
            continue;
        }
        
        const float ratioDist = dist2/dist1;
        const float ratioOctave = scaleFactorSet[kp1.octave] / scaleFactorSet[kp2.octave];
        
        /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
         continue;*/
        if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor) {
            successFlag.push_back(0);
            MESSAGE_PRINT("ratio distance out of range\n");
            continue;
        }
        
        // Triangulation is succesfull
        mapPoints.push_back(x3D);
        successFlag.push_back(1);

        MESSAGE_PRINT("step 5, success\n");
    }

    MESSAGE_PRINT("triangulate end, info detail: \n");
    MESSAGE_PRINT("input matched keys num:%d \n", currKeys.size());
    MESSAGE_PRINT("triangulate succes 3d points:%d \n", mapPoints.size());
}