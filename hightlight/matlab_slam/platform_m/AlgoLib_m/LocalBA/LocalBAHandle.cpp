//
//  LocalBAHandle.cpp
//  Matlab SLAM
//
//  Created by zhaiq on 7/18/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "LocalBAHandle.hpp"
#include <fstream>
#include "Converter.h"
#include "3rdparty/g2o/g2o/core/block_solver.h"
#include "3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "3rdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "3rdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "3rdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "3rdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "3rdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "utils.hpp"


float calcBAError(const std::vector<g2o::EdgeSE3ProjectXYZ*> &edgeList){
    float totatErr =0;
    for(int i=0;i<edgeList.size();i++){
        edgeList[i]->computeError();
        totatErr = totatErr+sqrt(edgeList[i]->chi2());
    }
    totatErr= totatErr/edgeList.size();
    return totatErr;
}

void localBAHandle(std::unordered_map<long, ygomi::Frame>& keyFrames,
                   std::unordered_map<long, ygomi::MapPoint>& mapPoints,
                   const cv::Mat& tK,
                   const std::vector<float>& invScaleFactorSigma2)
{
    // save("core_input_key_frame.txt", keyFrames);
    // save("core_input_map_point.txt", mapPoints);
    
	cv::Mat K;
    tK.convertTo(K, CV_32FC1);
        
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr                     = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver           = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    long maxKFid = 0;
    
    // Set key frames vertices
    for(auto it : keyFrames) {
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();

        const long& frameId         = it.first;
        const ygomi::Frame& frame   = it.second;
        
        vSE3->setEstimate(ORB_SLAM2::Converter::toSE3Quat(frame.pose));
        vSE3->setId(static_cast<int>(frameId)); //
        vSE3->setFixed(frameId==1);
        // vSE3->setFixed(true);

        optimizer.addVertex(vSE3);
        
        maxKFid = frameId > maxKFid ? frameId : maxKFid;
    }
    
    // Set map point vertices
    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    
    std::vector<long> mapPointsEdges;
    std::vector<long> keyFrameEdges;

    const float thHuberMono = sqrt(5.991);
    
    for(auto it : mapPoints) {
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        
        vPoint->setEstimate(ORB_SLAM2::Converter::toVector3d(it.second.pt)); //todo: check whether toVector3d accept cv::Point3f
        
        long id = it.first + maxKFid + 1;
        vPoint->setId(static_cast<int>(id));
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        
        const std::vector<ygomi::Track>& tracks = it.second.tracks;
        for(auto item : tracks) {
            if(!keyFrames.count(item.frameId))
                continue;
            const ygomi::Frame& frame = keyFrames[item.frameId];
            const cv::KeyPoint& kpUn  = frame.keyPoints[item.keyPointId].pt;
            
            Eigen::Matrix<double,2,1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;
            
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(id))));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(item.frameId))));
            e->setMeasurement(obs);
            
            const float &invSigma2 = invScaleFactorSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
            
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);
            
            e->fx = K.at<float>(0, 0);
            e->fy = K.at<float>(1, 1);
            e->cx = K.at<float>(0, 2);
            e->cy = K.at<float>(1, 2);
            
            optimizer.addEdge(e);
            
            vpEdgesMono.push_back(e);
            
            mapPointsEdges.push_back(it.first);
            keyFrameEdges.push_back(item.frameId);
        }
    }
    
    // std::cout << "before BA: " << calcBAError(vpEdgesMono) << std::endl;
    
    // Optimize
    optimizer.initializeOptimization();
    const int niterations = 5;
    optimizer.optimize(niterations);
    
    // std::cout << "after BA: " << calcBAError(vpEdgesMono) << std::endl;
    
    const bool doMore = true;
    if(doMore) {
        // Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++) {
            g2o::EdgeSE3ProjectXYZ* e       = vpEdgesMono[i];
            const ygomi::MapPoint& mapPoint = mapPoints[mapPointsEdges[i]];
            
            if(mapPoint.isBad)  continue;
            
            if(e->chi2()>5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }
        
        // std::cout << "before BA: " << calcBAError(vpEdgesMono) << std::endl;
        
        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
        
        // std::cout << "after BA: " << calcBAError(vpEdgesMono) << std::endl;
    }
    
    // earse
    std::vector<std::pair<long, long> > toErasedPairs;
    for(size_t i=0; i < vpEdgesMono.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e   = vpEdgesMono[i];
        ygomi::MapPoint mapPoint    = mapPoints[mapPointsEdges[i]];
        ygomi::Frame frame          = keyFrames[keyFrameEdges[i]];

        if(mapPoint.isBad) continue;
        
        if(e->chi2()>5.991 || !e->isDepthPositive()) {
            toErasedPairs.push_back(std::make_pair(keyFrameEdges[i], mapPointsEdges[i]));
        }
    }
    
    if(!toErasedPairs.empty()) {
        for(auto it : toErasedPairs) {
            //remove map point id of 2d keys.
            const long& frameId     = it.first;
            
            //remove item in track list.
            const long& mapPointId   = it.second;
            const std::vector<ygomi::Track>& tracks = mapPoints[mapPointId].tracks;
            std::vector<ygomi::Track> new_track;
            for(auto iter : tracks) {
                if(iter.frameId == frameId) {
                    keyFrames[frameId].keyPoints[iter.keyPointId].mapPointId = -1;
                }
                else {
                    new_track.push_back(iter);
                }
            }
            mapPoints[mapPointId].tracks = new_track;
        }
    }
    
    // update optimized data.
    for(std::unordered_map<long, ygomi::Frame>::iterator it = keyFrames.begin(), itEnd = keyFrames.end(); it != itEnd; it++) {
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(static_cast<int>(it->first)));
        g2o::SE3Quat SE3quat       = vSE3->estimate();
        
        const cv::Mat new_pose =  ORB_SLAM2::Converter::toCvMat(SE3quat);
        it->second.pose = new_pose.rowRange(0, 3);
    }
    
    float error = 0;
    for(std::unordered_map<long, ygomi::MapPoint>::iterator it = mapPoints.begin(); it != mapPoints.end(); it++) {
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(static_cast<int>(it->first + maxKFid + 1)));
        const cv::Mat point            = ORB_SLAM2::Converter::toCvMat(vPoint->estimate());
        
        it->second.pt                   = cv::Point3f(point.at<float>(0, 0), point.at<float>(1, 0), point.at<float>(2, 0));

        //pMP->UpdateNormalAndDepth(); //todo: update normal and depth of map point.
    }
    
    // save("core_output_frame.txt", keyFrames);
    // save("core_output_mappoint.txt", mapPoints);
}