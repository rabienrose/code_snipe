#include"Tracking.h"
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"ORBmatcher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"
#include"Optimizer.h"
#include"PnPsolver.h"
#include<iostream>

using namespace std;
namespace ORB_SLAM2
{
    Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath):
    mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpMap(pMap), mnLastRelocFrameId(0)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);
        
        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);
        
        mbf = fSettings["Camera.bf"];
        
        float fps = fSettings["Camera.fps"];
        if(fps==0)
            fps=30;
        
        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;
        
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];
        
        mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        mpIniORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }
    
    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }
    
    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing=pLoopClosing;
    }
    
    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
    {
        mImGray = im;
        
        if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
        }
        
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf, 0.0);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf, 0.0);
        
        Track();
        
        return mCurrentFrame.mTcw.clone();
    }
    
    void Tracking::Track()
    {
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }
        
        if(mState==NOT_INITIALIZED)
        {
            MonocularInitialization();
            if(mState!=OK)
                return;
        }
        else
        {
            bool bOK;
            if(mState==OK)
            {
                if(mCurrentFrame.mTimeStamp ==11){
                    mpReferenceKF = mpMap->GetKFbyId(1);
                }
                bOK = TrackReferenceKeyFrame();
            }
            else
            {
                bOK = Relocalization();
            }
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
            if(bOK)
                bOK = TrackLocalMap();
            if(bOK)
                mState = OK;
            else
                mState=LOST;
            if(bOK)
            {
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }
                
                // Delete temporal MapPoints
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();
                
                // Check if we need to insert a new keyframe
                if(NeedNewKeyFrame())
                    CreateNewKeyFrame();
                
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }
            
            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }
    
    
    void Tracking::MonocularInitialization()
    {
        
        if(!mpInitializer)
        {
            // Set Reference Frame
            if(mCurrentFrame.mvKeys.size()>100)
            {
                mInitialFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;
                
                if(mpInitializer)
                    delete mpInitializer;
                
                mpInitializer =  new Initializer(mCurrentFrame,1.0,4000);
                
                fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
                
                return;
            }
        }
        else
        {
            // Try to initialize
            if((int)mCurrentFrame.mvKeys.size()<=100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
                return;
            }
            
            // Find correspondences
            ORBmatcher matcher(0.9,true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
            // Check if there are enough correspondences
            std::cout<<"MonocularInitialization: "<<nmatches<<std::endl;
            if(nmatches<100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return;
            }
            
            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
            
            if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
            {
                for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                }
                
                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(Tcw.rowRange(0,3).col(3));
                mCurrentFrame.SetPose(Tcw);
                
                CreateInitialMapMonocular();
            }else{
                std::cout<<"initialier failed!!"<<std::endl;
            }
        }
    }
    
    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames
        KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        
        
        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();
        
        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);
        
        // Create MapPoints and asscoiate to keyframes
        for(size_t i=0; i<mvIniMatches.size();i++)
        {
            if(mvIniMatches[i]<0)
                continue;
            
            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);
            
            MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);
            
            pKFini->AddMapPoint(pMP,i);
            pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
            
            pMP->AddObservation(pKFini,i);
            pMP->AddObservation(pKFcur,mvIniMatches[i]);
            
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
            
            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;
            
            //Add to Map
            mpMap->AddMapPoint(pMP);
        }
        
        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();
        
        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
        
        Optimizer::GlobalBundleAdjustemnt(mpMap,20);
        
        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f/medianDepth;
        
        //    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
        //    {
        //        cout << "Wrong initialization, reseting..." << endl;
        //        Reset();
        //        return;
        //    }
        
        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        pKFcur->SetPose(Tc2w);
        
        // Scale points
        vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
        for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            }
        }
        
        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);
        
        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;
        
        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;
        
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        
        mState=OK;
    }
    
    bool Tracking::TrackReferenceKeyFrame()
    {
        mCurrentFrame.ComputeBoW();
        ORBmatcher matcher(0.7,true);
        vector<MapPoint*> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        int track_1 = mpReferenceKF->TrackedMapPoints(1);
        int track_2 = mpReferenceKF->TrackedMapPoints(2);
        int track_3 = mpReferenceKF->TrackedMapPoints(3);
        std::cout<<"kf_mp_count: "<<track_1<<":"<<track_2<<":"<<track_3<<std::endl;
        if(nmatches<15){
            std::cout<<"SearchByBoW in track ref: "<<nmatches<<std::endl;
            return false;
        }
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mpReferenceKF->GetPose());
        Optimizer::PoseOptimization(&mCurrentFrame);
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }
        if (nmatchesMap<10){
            std::cout<<"TrackReference: "<<nmatchesMap<<std::endl;
        }
        return nmatchesMap>=10;
    }
    
    bool Tracking::TrackLocalMap()
    {
        UpdateLocalMap();
        SearchLocalPoints();
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    mnMatchesInliers++;
                }
            }
        }
        if(mnMatchesInliers<30){
            std::cout<<"TrackLocalMap mp count: "<<mnMatchesInliers<<std::endl;
            return false;
        }
        else
            return true;
    }
    
    
    bool Tracking::NeedNewKeyFrame()
    {
        int nMinObs=3;
        if(mpMap->KeyFramesInMap()<=2){
            nMinObs= 2;
        }
        
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
        float thRefRatio = 0.9f;
        const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio) && mnMatchesInliers>15);
        if(c2)
        {
            return true;
        }
        else
            return false;
    }
    
    void Tracking::CreateNewKeyFrame()
    {
        KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;
        mpLocalMapper->InsertKeyFrame(pKF);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }
    
    void Tracking::SearchLocalPoints()
    {
        for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(pMP->isBad())
                {
                    *vit = static_cast<MapPoint*>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }
        int nToMatch=0;
        for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pMP->isBad())
                continue;
            if(mCurrentFrame.isInFrustum(pMP,0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }
        if(nToMatch>0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
    }
    
    void Tracking::UpdateLocalMap()
    {
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }
    
    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();
        
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
            
            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }
    }
    
    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame*,int> keyframeCounter;
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
        
        if(keyframeCounter.empty())
            return;
        
        int max=0;
        KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);
        
        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3*keyframeCounter.size());
        
        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFrame* pKF = it->first;
            
            if(pKF->isBad())
                continue;
            
            if(it->second>max)
            {
                max=it->second;
                pKFmax=pKF;
            }
            
            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }
        
        
        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if(mvpLocalKeyFrames.size()>80)
                break;
            
            KeyFrame* pKF = *itKF;
            
            const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
            
            for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
            {
                KeyFrame* pNeighKF = *itNeighKF;
                if(!pNeighKF->isBad())
                {
                    if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }
            
            const set<KeyFrame*> spChilds = pKF->GetChilds();
            for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
            {
                KeyFrame* pChildKF = *sit;
                if(!pChildKF->isBad())
                {
                    if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }
            
            KeyFrame* pParent = pKF->GetParent();
            if(pParent)
            {
                if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
            
        }
        
        if(pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }
    
    bool Tracking::Relocalization()
    {
        mCurrentFrame.ComputeBoW();
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
        if(vpCandidateKFs.empty())
            return false;
        const int nKFs = vpCandidateKFs.size();
        ORBmatcher matcher(0.75,true);
        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);
        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);
        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);
        int nCandidates=0;
        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);
        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;
                
                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;
                
                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);
                    
                    set<MapPoint*> sFound;
                    
                    const int np = vbInliers.size();
                    
                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }
                    
                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                    
                    if(nGood<10)
                        continue;
                    
                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);
                    
                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);
                        
                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                            
                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);
                                
                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                    
                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io]=NULL;
                                }
                            }
                        }
                    }
                    
                    
                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }
        
        if(!bMatch)
        {
            std::cout<<"ReLoc Fail!"<<std::endl;
            return false;
        }
        else
        {
            std::cout<<"ReLoc Succ!"<<std::endl;
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
        
    }
    
} //namespace ORB_SLAM
