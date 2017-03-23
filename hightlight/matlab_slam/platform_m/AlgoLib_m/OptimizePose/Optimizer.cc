#include "Optimizer.h"

#include<Eigen/StdVector>
#include <mutex>
#include <iostream>

#include "Converter.h"
#include "3rdparty/g2o/g2o/core/block_solver.h"
#include "3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "3rdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "3rdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "3rdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "3rdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "3rdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{
    static float calReprojError(const std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*>& vpEdgesMono){
        float totalErr=0;
        for(int i=0;i<vpEdgesMono.size();i++){
            vpEdgesMono[i]->computeError();
            totalErr=totalErr+vpEdgesMono[i]->chi2();
        }
        totalErr= totalErr/vpEdgesMono.size();
        return totalErr;
    }

    int Optimizer::PoseOptimization(const std::vector<float>& cameraParam,
                                    const cv::Mat& initialPose,
                                    const std::vector<cv::Point3f>& mapPoints,
                                    const std::vector<cv::KeyPoint>& keysIn2D,
                                    const std::vector<float>& mvInvLevelSigma2,
                                    cv::Mat& optimizedPose,
                                    std::vector<bool>& bOutliers)
    {
        assert(mapPoints.size() == keysIn2D.size());
        
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
        
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
        
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        
        int nInitialCorrespondences=0;
        
        // Set Frame vertex
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(initialPose));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);
        
        // Set MapPoint vertices
        const int N = static_cast<int>(mapPoints.size());
        
        std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        std::vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);
        
        const float deltaMono = sqrt(5.991);
        
        bOutliers.clear();
        bOutliers.reserve(mapPoints.size());

        for(int i=0; i<N; i++)
        {
            nInitialCorrespondences++;
            bOutliers.push_back(false);
            
            Eigen::Matrix<double,2,1> obs;
            const cv::KeyPoint &kpUn = keysIn2D[i];
            obs << kpUn.pt.x, kpUn.pt.y;
            
            g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
            
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
            
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);
            
            //load camera intrinsic param
            e->fx = cameraParam[0];
            e->fy = cameraParam[1];
            e->cx = cameraParam[2];
            e->cy = cameraParam[3];
            
            //load world coordinate position
            e->Xw[0] = mapPoints[i].x;
            e->Xw[1] = mapPoints[i].y;
            e->Xw[2] = mapPoints[i].z;
            
            optimizer.addEdge(e);
            
            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
        }
        
        
        if(nInitialCorrespondences<3)
            return 0;
        
        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] =   {5.991,5.991,5.991,5.991};
        const int its[4]        =   {10,10,10,10};
        
        int nBad=0;
        for(size_t it=0; it<4; it++)
        {
            
            vSE3->setEstimate(Converter::toSE3Quat(initialPose));
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);
            
            nBad=0;
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];
                
                const size_t idx = vnIndexEdgeMono[i];
                
                if(bOutliers[idx])
                {
                    e->computeError();
                }
                
                const float chi2 = e->chi2();
                
                if(chi2>chi2Mono[it])
                {                
                    bOutliers[idx]=true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    bOutliers[idx]=false;
                    e->setLevel(0);
                }
                
                if(it==2)
                    e->setRobustKernel(0);
            }
            
            if(optimizer.edges().size()<10)
                break;
        }    
        
        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        optimizedPose = pose.clone();
        
        return nInitialCorrespondences-nBad;
    }
} //namespace ORB_SLAM
