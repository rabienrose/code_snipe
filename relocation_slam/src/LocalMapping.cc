#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include <unistd.h>

namespace ORB_SLAM2
{
    
    LocalMapping::LocalMapping(Map *pMap): mpMap(pMap)
    {
    }
    
    void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
    {
        mpLoopCloser = pLoopCloser;
    }
    
    void LocalMapping::SetTracker(Tracking *pTracker)
    {
        mpTracker=pTracker;
    }
    
    void LocalMapping::Run()
    {
        ProcessNewKeyFrame();
        MapPointCulling();
        CreateNewMapPoints();
        SearchInNeighbors();
        if(mpMap->KeyFramesInMap()>2){
            bool bAbortBA=false;
            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&bAbortBA, mpMap);
        }
        KeyFrameCulling();
        //mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        //mpLoopCloser->Run();
    }
    
    void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        mlNewKeyFrames.push_back(pKF);
        Run();
    }

    void LocalMapping::ProcessNewKeyFrame()
    {
        {
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }
        
        // Compute Bags of Words structures
        mpCurrentKeyFrame->ComputeBoW();
        
        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }
        
        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();
        
        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }
    
    void LocalMapping::MapPointCulling()
    {
        // Check Recent Added MapPoints
        list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;
        
        int nThObs = 2;
        const int cnThObs = nThObs;
        
        while(lit!=mlpRecentAddedMapPoints.end())
        {
            MapPoint* pMP = *lit;
            if(pMP->isBad())
            {
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if(pMP->GetFoundRatio()<0.25f )
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
                lit++;
        }
    }
    
    void LocalMapping::CreateNewMapPoints()
    {
        int nn=20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        ORBmatcher matcher(0.6,false);
        cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
        cv::Mat Tcw1(3,4,CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0,3));
        tcw1.copyTo(Tcw1.col(3));
        cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
        
        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;
        
        const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
        
        int nnew=0;
        
        // Search matches with epipolar restriction and triangulate
        for(size_t i=0; i<vpNeighKFs.size(); i++)
        {
            
            KeyFrame* pKF2 = vpNeighKFs[i];
            
            // Check first that baseline is not too short
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            cv::Mat vBaseline = Ow2-Ow1;
            const float baseline = cv::norm(vBaseline);
            
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            if(ratioBaselineDepth<0.01)
                continue;
            cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);
            vector<pair<size_t,size_t> > vMatchedIndices;
            matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);
            cv::Mat Rcw2 = pKF2->GetRotation();
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslation();
            cv::Mat Tcw2(3,4,CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0,3));
            tcw2.copyTo(Tcw2.col(3));
            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;
            const int nmatches = vMatchedIndices.size();
            for(int ikp=0; ikp<nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;
                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
                const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);
                cv::Mat ray1 = Rwc1*xn1;
                cv::Mat ray2 = Rwc2*xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
                float cosParallaxStereo = cosParallaxRays+1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;
                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);
                cv::Mat x3D;
                if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (cosParallaxRays<0.9998))
                {
                    cv::Mat A(4,4,CV_32F);
                    A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
                    
                    cv::Mat w,u,vt;
                    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
                    
                    x3D = vt.row(3).t();
                    
                    if(x3D.at<float>(3)==0)
                        continue;
                    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
                }
                else
                    continue;
                
                cv::Mat x3Dt = x3D.t();
                float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
                if(z1<=0)
                    continue;
                
                float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                if(z2<=0)
                    continue;
                
                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
                const float invz1 = 1.0/z1;
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>7.8*sigmaSquare1)
                    continue;
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
                const float invz2 = 1.0/z2;
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>7.8*sigmaSquare2)
                    continue;
                
                //Check scale consistency
                cv::Mat normal1 = x3D-Ow1;
                float dist1 = cv::norm(normal1);
                
                cv::Mat normal2 = x3D-Ow2;
                float dist2 = cv::norm(normal2);
                
                if(dist1==0 || dist2==0)
                    continue;
                
                const float ratioDist = dist2/dist1;
                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];
                
                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                 continue;*/
                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                    continue;
                
                // Triangulation is succesfull
                MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
                
                pMP->AddObservation(mpCurrentKeyFrame,idx1);
                pMP->AddObservation(pKF2,idx2);
                
                mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
                pKF2->AddMapPoint(pMP,idx2);
                
                pMP->ComputeDistinctiveDescriptors();
                
                pMP->UpdateNormalAndDepth();
                
                mpMap->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.push_back(pMP);
                
                nnew++;
            }
        }
        if(nnew>0){
            std::cout<<nnew<<" new mps created"<<std::endl;
        }
    }
    
    void LocalMapping::SearchInNeighbors()
    {
        int nn = 20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        vector<KeyFrame*> vpTargetKFs;
        for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
            
            // Extend to some second neighbors
            const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
            {
                KeyFrame* pKFi2 = *vit2;
                if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
            }
        }
        
        
        // Search matches by projection from current KF in target KFs
        ORBmatcher matcher;
        vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            
            matcher.Fuse(pKFi,vpMapPointMatches);
        }
        
        // Search matches by projection from target KFs in current KF
        vector<MapPoint*> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());
        
        for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        {
            KeyFrame* pKFi = *vitKF;
            
            vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
            
            for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
            {
                MapPoint* pMP = *vitMP;
                if(!pMP)
                    continue;
                if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }
        
        matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
        
        
        // Update points
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
        {
            MapPoint* pMP=vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    pMP->ComputeDistinctiveDescriptors();
                    pMP->UpdateNormalAndDepth();
                }
            }
        }
        
        // Update connections in covisibility graph
        mpCurrentKeyFrame->UpdateConnections();
    }
    
    cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
    {
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();
        
        cv::Mat R12 = R1w*R2w.t();
        cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
        
        cv::Mat t12x = SkewSymmetricMatrix(t12);
        
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;
        
        
        return K1.t().inv()*t12x*R12*K2.inv();
    }
    
    void LocalMapping::KeyFrameCulling()
    {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points
        vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
        
        for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            if(pKF->mnId==0)
                continue;
            const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
            
            int nObs = 3;
            const int thObs=nObs;
            int nRedundantObservations=0;
            int nMPs=0;
            for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
            {
                MapPoint* pMP = vpMapPoints[i];
                if(pMP)
                {
                    if(!pMP->isBad())
                    {
                        nMPs++;
                        if(pMP->Observations()>thObs)
                        {
                            const int &scaleLevel = pKF->mvKeysUn[i].octave;
                            const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                            int nObs=0;
                            for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFrame* pKFi = mit->first;
                                if(pKFi==pKF)
                                    continue;
                                const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;
                                
                                if(scaleLeveli<=scaleLevel+1)
                                {
                                    nObs++;
                                    if(nObs>=thObs)
                                        break;
                                }
                            }
                            if(nObs>=thObs)
                            {
                                nRedundantObservations++;
                            }
                        }
                    }
                }
            }  
            
            if(nRedundantObservations>0.9*nMPs)
                pKF->SetBadFlag();
        }
    }
    
    cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2),               0,-v.at<float>(0),
                -v.at<float>(1),  v.at<float>(0),              0);
    }
} //namespace ORB_SLAM
