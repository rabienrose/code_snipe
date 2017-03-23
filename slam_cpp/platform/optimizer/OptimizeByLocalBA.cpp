/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   OptimizeByLocalBA.cpp
 * @brief  Implementation of the interface for optimization using local Bandle 
 *         Adjustment.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.08        Qiang.Zhai     Create
 *******************************************************************************
 */
#include <unordered_map>
#include "OptimizeByLocalBA.hpp"
#include "../../../3rdparty/g2o/g2o/core/block_solver.h"
#include "../../../3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../../../3rdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../../../3rdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../../../3rdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../../../3rdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "../../../3rdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Utils.h"
#include "TypeDef.hpp"

float calcBAError(const std::vector<g2o::EdgeSE3ProjectXYZ*> &edgeList){
    float totatErr =0;
    for(int i=0;i<edgeList.size();i++){
        edgeList[i]->computeError();
        totatErr = totatErr+sqrt(edgeList[i]->chi2());
    }
    totatErr= totatErr/edgeList.size();
    return totatErr;
}

void optimizeByLocalBA(std::unordered_map<long, ygomi::Frame*>& keyFrames,
                       std::unordered_map<long, ygomi::MapPoint*>& mapPoints,
                       const cv::Mat& tK,
                       const std::vector<float>& invScaleFactorSigma2, int nIterationNum)
{
    cv::Mat K;
    tK.convertTo(K, CV_32FC1);
    
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr                     = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver           = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    long maxKFid = 0;
    
    static bool bIsFirstFrameIDFound = false;
    static long nFirstFrameID = 0XFFFFFFFF;
    if(!bIsFirstFrameIDFound) {
        for(const auto& it : keyFrames) {
            if(nFirstFrameID > it.first) {
                nFirstFrameID        = it.first;
                bIsFirstFrameIDFound = true;
            }
        }
    }
    
    // Set key frames vertices
    for(const auto& it : keyFrames) {
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        
        const long& frameId         = it.first;
        const ygomi::Frame* frame   = it.second;
        
        cv::Mat pose = frame->getPose().clone();        
        pose.convertTo(pose, CV_32FC1);
        vSE3->setEstimate(algo::toSE3Quat(pose));
        vSE3->setId(static_cast<int>(frameId)); //
        vSE3->setFixed(frameId==nFirstFrameID);
        
        optimizer.addVertex(vSE3);
        
        maxKFid = frameId > maxKFid ? frameId : maxKFid;
    }
    
    // Set map point vertices
    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    
    std::vector<long> mapPointsEdges;
    std::vector<long> keyFrameEdges;
    
    const float thHuberMono = sqrt(5.991);
    
    for(const auto& it : mapPoints) {
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        
        vPoint->setEstimate(algo::toVector3d(it.second->getPosition()));
        
        long id = it.first + maxKFid + 1;
        vPoint->setId(static_cast<int>(id));
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        
        const std::vector<ygomi::Track>& tracks = it.second->getObservation();
        for(const auto& item : tracks) {
            if(!keyFrames.count(item.m_frameId))
                continue;
            const ygomi::Frame* frame = keyFrames.find(item.m_frameId)->second;
            const cv::KeyPoint& kpUn  = frame->getKeyPoint(item.m_keyPointId).m_key;
            
            Eigen::Matrix<double,2,1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;
            
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(id))));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(static_cast<int>(item.m_frameId))));
            e->setMeasurement(obs);
            
            const float &invSigma2 = invScaleFactorSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * tracks.size());
            
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
            keyFrameEdges.push_back(item.m_frameId);
        }
    }
    
    // std::cout << "before BA: " << calcBAError(vpEdgesMono) << std::endl;
    
    // Optimize
    optimizer.initializeOptimization();
//    const int niterations = 5;
    optimizer.optimize(nIterationNum);
    
    // std::cout << "after BA: " << calcBAError(vpEdgesMono) << std::endl;
    
    const bool doMore = true;
    if(doMore) {
        // Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++) {
            g2o::EdgeSE3ProjectXYZ* e       = vpEdgesMono[i];
            const ygomi::MapPoint* mapPoint = mapPoints.find(mapPointsEdges[i])->second;
            
            if(mapPoint->isBad())  continue;
            
            if(e->chi2()>5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }
        
        // std::cout << "before BA: " << calcBAError(vpEdgesMono) << std::endl;
        
        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(nIterationNum);
        
        // std::cout << "after BA: " << calcBAError(vpEdgesMono) << std::endl;
    }
    
    // earse
    std::vector<std::pair<long, long> > toErasedPairs;
    for(size_t i=0; i < vpEdgesMono.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e   = vpEdgesMono[i];
        ygomi::MapPoint* mapPoint    = mapPoints.find(mapPointsEdges[i])->second;
        //ygomi::Frame frame          = keyFrames[keyFrameEdges[i]];
        
        if(mapPoint->isBad()) continue;
        
        if(e->chi2()>5.991 || !e->isDepthPositive()) {
            toErasedPairs.push_back(std::make_pair(keyFrameEdges[i], mapPointsEdges[i]));
        }
    }
    
    if(!toErasedPairs.empty()) {
        for(const auto& it : toErasedPairs) {
            //remove map point id of 2d keys.
            const long& frameId     = it.first;
            ygomi::Frame* frame     = keyFrames.find(frameId)->second;
            
            //remove item in track list.
            const long& mapPointId  = it.second;
            ygomi::MapPoint* mp     = mapPoints.find(mapPointId)->second;
            
            const std::vector<ygomi::Track>& tracks = mp->getObservation();
            std::vector<ygomi::Track> new_tracks;
            for(const auto& track : tracks) {
                if(track.m_frameId == frameId) {
                    frame->delMatchingPair(track.m_keyPointId, mapPointId);
                }
                else {
                    new_tracks.push_back(track);
                }
            }
            
            //update tracks.
            mp->removeAllTracks();
            mp->addTracks(new_tracks);
        }
    }
    
    // update optimized data.
    for(auto& item : keyFrames) {
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(static_cast<int>(item.first)));
        g2o::SE3Quat SE3quat       = vSE3->estimate();
        
        cv::Mat new_pose =  algo::toCvMat(SE3quat);
        new_pose.convertTo(new_pose, CV_64FC1);
        item.second->setPose(new_pose.rowRange(0, 3));
        
    }

    for(auto& item : mapPoints) {
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(static_cast<int>(item.first + maxKFid + 1)));
        const cv::Mat point            = algo::toCvMat(vPoint->estimate());
        
        item.second->setPosition(cv::Point3f(point.at<float>(0, 0), point.at<float>(1, 0), point.at<float>(2, 0)));
        
        //pMP->UpdateNormalAndDepth(); //todo: update normal and depth of map point.
    }
}