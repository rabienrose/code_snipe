#include "AlgoPlat_G2O.h"

#include "Visualisation/ViewInterfaceImp.h"
#include "CoreData.h"
#include "Configuration.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Utils.h"

namespace chamo{
    
    float AlgoPlat_G2O::calBAError(std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> &edgeList){
        float totatErr =0;
        for(int i=0;i<edgeList.size();i++){
            edgeList[i]->computeError();
            totatErr = totatErr+sqrt(edgeList[i]->chi2());
        }
        totatErr= totatErr/edgeList.size();
        return totatErr;
    }
    
    float AlgoPlat_G2O::calBAError(std::vector<g2o::EdgeSE3ProjectXYZ*> &edgeList){
        float totatErr =0;
        for(int i=0;i<edgeList.size();i++){
            edgeList[i]->computeError();
            totatErr = totatErr+sqrt(edgeList[i]->chi2());
        }
        totatErr= totatErr/edgeList.size();
        return totatErr;
    }
    
    
    float AlgoPlat_G2O::PoseOptimize(std::vector<cv::Mat>& mpList, std::vector<cv::KeyPoint>& kpList, cv::Mat& PoseW, cv::Mat K, cv::Mat gpsPosi){
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        int mpOffset = 10000;
        const float thHuberMono = sqrt(5.991);
        
        g2o::VertexSE3Expmap * vSE3cur = new g2o::VertexSE3Expmap();
        vSE3cur->setEstimate( algo::toSE3Quat(PoseW));
        vSE3cur->setId(0);
        vSE3cur->setFixed(false);
        optimizer.addVertex(vSE3cur);
        
        std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        for(int i=0;i<kpList.size();i++){
            Eigen::Matrix<double,2,1> obs;
            obs << kpList[i].pt.x, kpList[i].pt.y; //keypoint
            
            g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());
            
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);
            
            e->fx = K.at<double>(0,0);
            e->fy = K.at<double>(1,1);
            e->cx = K.at<double>(0,2);
            e->cy = K.at<double>(1,2);
            e->Xw[0] = mpList[i].at<float>(0);
            e->Xw[1] = mpList[i].at<float>(1);
            e->Xw[2] = mpList[i].at<float>(2);
            e->computeError();
            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
        }
        
        if(!gpsPosi.empty()){
            std::vector<g2o::EdgeSE3ProjectXYZ *> vEdge;
            g2o::VertexSE3Expmap * vGps = new g2o::VertexSE3Expmap();
            const int idGps = 1;
            vGps->setId(idGps);
            vGps->setFixed(true);
            optimizer.addVertex(vGps);
            
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idGps)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            
            Eigen::Matrix<double,2,1> obs;
            obs << 0.0, 0.0;
            e->setMeasurement(obs);
            
            const float beta = 1e-6;
            e->setInformation(Eigen::Matrix2d::Identity()*beta);
            optimizer.addEdge(e);
            vEdge.push_back(e);
            e->computeError();
        }
        float err;
        
        //float err =calBAError(vpEdgesMono);
        //std::cout<<"err before ba: "<<err<<std::endl;
        
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        
        err =calBAError(vpEdgesMono);
        //std::cout<<"err after ba: "<<err<<std::endl;
        
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        PoseW =algo::toCvMat(SE3quat);
        
        return err;
    }
    
    float AlgoPlat_G2O::doBA(int curKF, int offset, bool showErr){
        int endKF =curKF;
        int startKF = curKF+offset;
        if (startKF <0){
            startKF =0;
        }
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        int mpOffset = 10000;
        const float thHuberMono = sqrt(5.991);
        
        for(int i=startKF;i<=endKF;i++){
            g2o::VertexSE3Expmap * vSE3cur = new g2o::VertexSE3Expmap();
            vSE3cur->setEstimate( algo::toSE3Quat(coreData->getKFObj(i)->poseWorld ));
            vSE3cur->setId(i);
            vSE3cur->setFixed(false);
            optimizer.addVertex(vSE3cur);
            
        }
        
        std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
        for(int i=startKF;i<=endKF;i++){
            for (int j = 0; j<coreData->getKPCount(i);j++){
                int mapId = coreData->getKPObj(i, j)->TrackId;
                if(!coreData->getMPObj(mapId)->hasPosi()){
                    continue;
                }
                if(optimizer.vertex(mapId+mpOffset)!= NULL){
                    continue;
                }
                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(algo::toVector3d(coreData->getMPObj(mapId)->posi));
                vPoint->setId(mapId+mpOffset);
                vPoint->setMarginalized(true);
                optimizer.addVertex(vPoint);
                std::vector<TrackItemChamo>& track = coreData->getTrack(mapId);
                for(int k=0;k<track.size();k++){
                    if(track[k].frameId>=startKF && track[k].frameId<=endKF){
                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint *kp = coreData->getKPByTrack(track[k]);
                        obs << kp->pt.x, kp->pt.y; //keypoint
                        
                        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                        
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mapId+mpOffset)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(track[k].frameId)));
                        e->setMeasurement(obs);
                        e->setInformation(Eigen::Matrix2d::Identity());
                        
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);
                        
                        e->fx = setting->getK().at<double>(0,0);
                        e->fy = setting->getK().at<double>(1,1);
                        e->cx = setting->getK().at<double>(0,2);
                        e->cy = setting->getK().at<double>(1,2);
                        e->computeError();
                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                    }
                }
            }
        }
        float err;
        if (showErr){
            err =calBAError(vpEdgesMono);
            std::cout<<"err before ba: "<<err<<std::endl;
        }
        
        optimizer.initializeOptimization();
        optimizer.optimize(5);
        
        err =calBAError(vpEdgesMono);
        if (showErr){
            std::cout<<"err after ba: "<<err<<std::endl;
        }
        
        for(int i=0; i<vpEdgesMono.size(); i++)
        {
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(vpEdgesMono[i]->vertex(0));
            algo::toCvMat(vPoint->estimate()).copyTo(coreData->getMPObj(vPoint->id()-mpOffset)->posi.rowRange(0, 3));
        }
        
        for(int i=startKF;i<=endKF;i++)
        {
            g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            coreData->getKFObj(i)->poseWorld =algo::toCvMat(SE3quat);
        }
        return err;
    }
    
    void AlgoPlat_G2O::optimizeTOneF(int frameId){
        std::vector<cv::Mat> mpList;
        std::vector<cv::KeyPoint> kpList;
        std::vector<bool> inliers;
        for(int j=0;j<coreData->getKPCount(frameId);j++){
            int mapId = coreData->getKPObj(frameId, j)->TrackId;
            if(!coreData->getMPObj(mapId)->isBad && !coreData->getMPObj(mapId)->posi.empty()){
                mpList.push_back(coreData->getMPObj(mapId)->posi.rowRange(0, 3));
                kpList.push_back(*coreData->getKP(frameId, j));
            }
        }
        if(mpList.size()>0){
            cv::Mat lPose = coreData->getKFObj(frameId)->poseLocal;
            cv::Mat poselast = coreData->getKFObj(frameId-1)->poseWorld;
            cv::Mat poselastlast = coreData->getKFObj(frameId-2)->poseWorld;
            cv::Mat deltaPose =poselast *poselastlast.inv();
            cv::Mat localtlast = algo::getT(deltaPose);
            //std::cout<<"norm(localtlast): "<<cv::norm(localtlast) <<std::endl;
            cv::Mat localt = algo::getT(lPose);
            localt =localt*cv::norm(localtlast);
            cv::Mat rLast = algo::getR(lPose);
            cv::Mat estPose = algo::makeT(rLast, localt);
            estPose = deltaPose;
            estPose = estPose * coreData->getKFObj(frameId-1)->poseWorld;
            PoseOptimize(mpList, kpList, estPose, setting->getK(), cv::Mat());
            coreData->getKFObj(frameId)->poseWorld = estPose;
            calMPNViewTriang(frameId);
        }
    }
    

}
