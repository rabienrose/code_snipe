#include <mex.h>
#include <opencvmex.hpp>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "Optimizer.h"
#include "typeDef.hpp"

template<typename T>
static void printMat(const std::string& tag, const cv::Mat& m)
{
    MESSAGE_PRINT("%s\n", tag.c_str());
    for(int j=0; j<m.rows; j++) {
        for(int i=0; i<m.cols; i++) {
            MESSAGE_PRINT("%f ", m.at<T>(j, i));
        }
        MESSAGE_PRINT("\n");
    }
    MESSAGE_PRINT("\n");
}

static bool verify( const cv::Mat& K,
                    const cv::Mat& pose,
                    const std::vector<cv::Point3f>& points3D,
                    const std::vector<cv::KeyPoint>& currKeys,
                    const std::vector<float>& scaleSet,
                    const std::vector<float>& invScaleSet)
{
    //log
    printMat<double>("input camera param", K);

    printMat<float>("input camera pose", pose);

    //verify 3d points.
    MESSAGE_PRINT("input 3d keys: \n");
    for(int k=0; k<5 && k<points3D.size(); k++) {
        MESSAGE_PRINT("%f, %f, %f\n", points3D[k].x, points3D[k].y, points3D[k].z);
    }
    MESSAGE_PRINT("\n");

    //verify 2d keys.
    MESSAGE_PRINT("input 2d keys: \n");
    for(int k=0; k<5 && k<currKeys.size(); k++) {
        MESSAGE_PRINT("%f, %f, %d\n", currKeys[k].pt.x, currKeys[k].pt.y, currKeys[k].octave);
    }
    MESSAGE_PRINT("\n");

    //verify scale factor set.
    MESSAGE_PRINT("input scale factor set: \n");
    for(auto k=0; k<scaleSet.size(); k++) {
        MESSAGE_PRINT("%f ", scaleSet[k]);
    }
    MESSAGE_PRINT("\n");

    //verify calaulted scale factor.
    MESSAGE_PRINT("inversed scale factor set: \n");
    for(auto k=0; k<invScaleSet.size(); k++) {
        MESSAGE_PRINT("%f ", invScaleSet[k]);
    }
    MESSAGE_PRINT("\n");

    //
    if(K.cols != 3 || K.rows != 3)
        return false;
    if(pose.rows != 3 || pose.cols != 4)
        return false;
    if(points3D.empty() || currKeys.empty())
        return false;
    if(scaleSet.empty())
        return false;

    return true;
}

/**
 * Optimize pose based on 3D-2D matching pairs
 *@param prhs[0] Camera instrinsic parameters, 3 x 3 matrix.
 *@param prhs[1] Un-optimized camera pose, 3 x 4 matrix.
 *@param prhs[2] Coordinate of points 3D.
 *@param prhs[3] Coordinate of points 2D.
 *@param prhs[4] Scale factor set of points 2D.
 *@param plhs[0] [output] Optimized pose return to caller.
 *@param plhs[1] [output] Outlier flag return to caller.
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    MESSAGE_PRINT("input param num: %d\n", nrhs);
    MESSAGE_PRINT("output param num: %d\n", nlhs);
    
    assert(nrhs == 5 && nlhs == 2);
    
    ///parse input parameters.
    //read camera intrinsic parameters
    cv::Mat K;
    ocvMxArrayToImage_double(prhs[0], K);
    
    float fx = K.at<double>(0, 0);
    float fy = K.at<double>(1, 1);
    float cx = K.at<double>(0, 2);
    float cy = K.at<double>(1, 2);
    
    //read initial pose
    cv::Mat pose;
    ocvMxArrayToImage_double(prhs[1], pose);
    pose.convertTo(pose, CV_32FC1);
     
    //read map points
    cv::Mat mapPoints;
    ocvMxArrayToImage_double(prhs[2], mapPoints);
    //mapPoints = mapPoints.t();
    std::vector<cv::Point3f> points3D;
    points3D.reserve(mapPoints.rows);
    const double* ptr = nullptr;
    for(int i=0; i<mapPoints.rows; i++) {
        ptr = mapPoints.ptr<double>(i);
        points3D.push_back(cv::Point3f(*ptr++, *ptr++, *ptr));
    }
        
    //read matched 2d points
    std::vector<cv::KeyPoint> currKeys;
    ocvStructToKeyPoints(prhs[3], currKeys);
    assert(currKeys.size() == mapPoints.rows);

    //read inversed level sigma
    cv::Mat scaleSet;
    ocvMxArrayToImage_double(prhs[4], scaleSet);

    int nLevel = scaleSet.cols;
    std::vector<float> invScaleSet;
    invScaleSet.reserve(nLevel);
    for(int i=0; i<nLevel; i++) {
        float inv_scale = 1.0f/scaleSet.at<double>(0,i);
        invScaleSet.push_back(inv_scale * inv_scale);
    }

    //verify input parameters.
    if(!verify(K, pose, points3D, currKeys, scaleSet, invScaleSet)) {
        for(int i=0; i<nlhs; i++) {
            plhs[i] = mxCreateDoubleMatrix(1, 1, mxREAL);
        }
        ERROR_LOG("invalid input in mex while optimizing pose.\n");
        return;
    }

    ///optimize pose
    const int keyNum = currKeys.size();
    std::vector<bool> outliers(keyNum, false);
    std::vector<float> cameraParam = {fx, fy, cx, cy};
    cv::Mat updatedPose;
    try {
        ORB_SLAM2::Optimizer::PoseOptimization(
                                            cameraParam,     //camera intrinsic parameters
                                            pose,            //initial coarse pose
                                            points3D,        //map points position
                                            currKeys,        //keys in 2D after undistortion
                                            invScaleSet,     //scale lavel
                                            updatedPose,     //optimized pose
                                            outliers);       //outliers flag
    } catch(...) {
        ERROR_LOG("Caught a general exception in optimizePose.");
    }


    ///pass optimized pose to caller.
    //return optimized pose
    plhs[0] = ocvMxArrayFromMat_single(updatedPose);

    //return outlier flag
    plhs[1] = mxCreateDoubleMatrix((mwSize)keyNum, (mwSize)1, mxREAL);
    double* val = mxGetPr(plhs[1]);
    assert(val);
    for(int i=0 ; i<keyNum ; i++)
        val[i] = static_cast<double>(outliers[i]);
}