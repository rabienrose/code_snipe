
#include <string>
#include "Platform.hpp"
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "GlobalMap.hpp"
#include "TypeDef.hpp"
#include "searchByProjection.hpp"
#include "paramParser.hpp"
#include "Utils.h"
#include <fstream>


//#define RUN_TEST

#ifdef RUN_TEST
    #include <gtest/gtest.h>
#endif

extern void assignFeaturesToGrid(const std::vector<ygomi::KeyPoint>& frameKeys);
extern void computeImageBounds(const cv::Rect& imgRect, const cv::Mat& mK, const cv::Mat& mDistCoef);
extern void getFeaturesIndexInArea(const std::vector<cv::KeyPoint>& frameKeys, const cv::Point2f& pt, int r, std::vector<size_t>& indices, const int minLevel, const int maxLevel);
extern void getFeaturesIndexInArea(const std::vector<cv::KeyPoint>& frameKeys, const cv::Point2f& pt, int r, std::vector<size_t>& indices);


//1st
extern void readSingleFrame(std::ifstream& fin,
                     long& frameId,
                     ygomi::FrameType& type,
                     cv::Mat& pose,
                     long& refID,
                     std::vector<ygomi::KeyPoint>& keys,
                     cv::Point3f& cameraCenter,
                     std::string& name);

extern void readSingleMapPoint(std::ifstream& fin,
                        bool& isBad,
                        long& mpId,
                        cv::Point3f& mp_pt,
                        cv::Mat& descriptor,
                        std::vector<ygomi::Track>& tracks,
                        long& referKFID,
                        long& firstKFID,
                        float& minDistance,
                        float& maxDistance,
                        cv::Mat& normalVector,
                        int& visible,
                        int& found,
                        std::vector<cv::Point3f>& cameraCenters,
                        int& nlevel,
                        std::vector<float>& scaleFactors,
                        bool readExtra = false);

//2nd
extern void readSingleFrame(const std::string& path,
                     long& frameId,
                     ygomi::FrameType& type,
                     cv::Mat& pose,
                     long& refID,
                     std::vector<ygomi::KeyPoint>& keys,
                     cv::Point3f& cameraCenter,
                     std::string& name);

extern void readSingleMapPoint(const std::string& path,
                        bool& isBad,
                        long& mpId,
                        cv::Point3f& mp_pt,
                        cv::Mat& descriptor,
                        std::vector<ygomi::Track>& tracks,
                        long& referKFID,
                        long& firstKFID,
                        float& minDistance,
                        float& maxDistance,
                        cv::Mat& normalVector,
                        int& visible,
                        int& found,
                        std::vector<cv::Point3f>& cameraCenters,
                        int& nlevel,
                        std::vector<float>& scaleFactors, bool readExtra);

//3rd
extern ygomi::Frame* readSingleFrame(const std::string& path);
extern ygomi::MapPoint* readSingleMapPoint(const std::string& path);

//4th
extern ygomi::Frame* readSingleFrame(std::ifstream& fin);
extern ygomi::MapPoint* readSingleMapPoint(std::ifstream& fin);

//5th
extern std::vector<ygomi::Frame*> readAllFrames(const std::string& path);
extern std::vector<ygomi::MapPoint*> readAllMapPoints(const std::string& path);

namespace ygomi {
    bool Platform::DelMpKpConnection(long mpId, long frameId, int kpId){
        MapPoint* mp= m_pGlobalMap->getMapPoint(mpId);
        CV_Assert(mp);
        
        Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        
        frame->delMatchingPair(kpId, mpId);
        mp->removeTrack(Track(frameId, kpId));
        return true;
    }
    
    bool Platform::AddMpKpConnection(long mpId, long frameId, int kpId){
        MapPoint* mp= m_pGlobalMap->getMapPoint(mpId);
        CV_Assert(mp);
        
        Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        
        frame->addMatchingPair(kpId, mpId);
        mp->addTrack(frameId, kpId);
        //todo: calculate descriptor.
        return true;
    }
    
    std::vector<std::pair<int, int>> Platform::get2DMatches(long reFrameId, long toFrameId)
    {
        std::vector<std::pair<int, int>> matches;
        Frame* reFrame = m_pGlobalMap->getFrame(reFrameId);
        std::vector<std::pair<int, int>> vecMatches;
        const std::vector<ygomi::KeyPoint>& vecKps = reFrame->getKeyPoints();
        for(int i = 0; i < vecKps.size(); i++)
        {
            int kpmpCount = vecKps[i].m_mapPointId.size();
            if(kpmpCount <= 0)
            {
                continue;
            }
            for (int j = 0; j < kpmpCount; j++)
            {
                int mpId = vecKps[i].m_mapPointId[j];
                MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
                const std::vector<ygomi::Track>& track = mp->getObservation();
                for(int k = 0; k < track.size(); k++)
                {
                    if(track[k].m_frameId == toFrameId)
                    {
                        matches.push_back(std::pair<int, int>(i, track[k].m_keyPointId));
                        break;
                    }
                }
            }
        }
        return matches;
    }
    
    void Platform::searchByProjectionBack(long frameId, int keyFrameCount)
    {
        
    }
    
    template<typename T>
    cv::Point3f projectWorld2Camera(const cv::Mat& pose, const cv::Point3f& p3Dw)
    {
        cv::Point3f p3Dc;
        
        p3Dc.x = pose.ptr<T>(0)[0]*p3Dw.x + pose.ptr<T>(0)[1]*p3Dw.y + pose.ptr<T>(0)[2]*p3Dw.z + pose.ptr<T>(0)[3];
        p3Dc.y = pose.ptr<T>(1)[0]*p3Dw.x + pose.ptr<T>(1)[1]*p3Dw.y + pose.ptr<T>(1)[2]*p3Dw.z + pose.ptr<T>(1)[3];
        p3Dc.z = pose.ptr<T>(2)[0]*p3Dw.x + pose.ptr<T>(2)[1]*p3Dw.y + pose.ptr<T>(2)[2]*p3Dw.z + pose.ptr<T>(2)[3];
        
        return p3Dc;
    }
    
    template<typename T>
    cv::Point2f projectCamera2Image(const cv::Mat& K, const cv::Point3f& p3Dc)
    {
        T rc[3];
        for(int i=0; i<3; i++) {
            rc[i] = K.ptr<T>(i)[0]*p3Dc.x + K.ptr<T>(i)[1]*p3Dc.y + K.ptr<T>(i)[2]*p3Dc.z;
        }
        
        return cv::Point2f(rc[0]/rc[2], rc[1]/rc[2]);
    }
    
    inline bool inBoundary(int low_x, int low_y, int up_x, int up_y, int x, int y)
    {
        return x >= low_x && x < up_x && y >= low_y && y < up_y;
    }
    
    void Platform::mergeMapPoints(long frameId, size_t keyId)
    {
        //NOTE!!! The main merge strategy is that we retain that map point which
        //has the longest tracks, remove others, and update the connections.
        
        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame && frame->getType() != ygomi::NORMAL_FRAME);
        
        const ygomi::KeyPoint& key = frame->getKeyPoint(keyId); //will be updated.
        if(key.m_mapPointId.size() <= 1)
            return;
        
        std::vector<int> tracksNum;
        for(auto& mpId : key.m_mapPointId) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
            CV_Assert(mp && !mp->isBad());
            
            const auto& tracks = mp->getObservation();
            tracksNum.push_back(tracks.size());
        }
        
        //find the longest track to retain.
        long nKeepIdx = std::distance(tracksNum.begin(), std::max_element(tracksNum.begin(), tracksNum.end()));
        ygomi::MapPoint* new_mp = m_pGlobalMap->getMapPoint(key.m_mapPointId[nKeepIdx]);
        std::vector<ygomi::Track> new_tracks = new_mp->getObservation();
        
        //calculate weighted positon for map point.
        std::vector<float> weights;
        float sumWeights = 0.0;
        const int track_count = static_cast<const int>(new_tracks.size());
        for(auto& num : tracksNum) {
            float w = num * 1.0f / track_count;
            weights.push_back(w);
            sumWeights += w;
        }
        for(auto& w : weights) {
            w = w / sumWeights;
        }// normlize
        cv::Point3f new_pos;
        for(size_t id=0; id<key.m_mapPointId.size(); id++) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(key.m_mapPointId[id]);
            new_pos += mp->getPosition() * weights[id];
        }
        new_mp->setPosition(new_pos);
        
        //update visible and found.
        int nVisibile = 0, nRealFound = 0;
        for(auto& mpId : key.m_mapPointId) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
            nVisibile  += mp->getVisible();
            nRealFound += mp->getRealFould();
        }
        new_mp->addVisible(-new_mp->getVisible());
        new_mp->addVisible(nVisibile);
        new_mp->addFound(-new_mp->getRealFould());
        new_mp->addFound(nRealFound);
        
        // combination map points.
        for(auto& mpId : key.m_mapPointId) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
            CV_Assert(mp && !mp->isBad());
            if(mp->getId() == new_mp->getId())
                continue;
            
            const auto& tracks = mp->getObservation();
            for(const auto& track : tracks) {
                ///1.remove map point connections of other frames.
                ygomi::Frame* other_frame = m_pGlobalMap->getFrame(track.m_frameId);
                CV_Assert(other_frame);
                
                const ygomi::KeyPoint& other_key = other_frame->getKeyPoint(track.m_keyPointId);
                const auto& mpIndices = other_key.m_mapPointId;
                
                //connection of key may not be unique.
                size_t idx = std::distance(mpIndices.begin(), std::find(mpIndices.begin(), mpIndices.end(), mp->getId()));
                CV_Assert(idx < mpIndices.size()); //must exist.
                
                other_frame->delMatchingPair(track.m_keyPointId, mpIndices[idx]);
                
                ///2.add current track to tracks set, and ignore track alreadly exist.
                bool bExist = false;
                for(const auto& new_t : new_tracks) {
                    if(new_t.m_frameId == track.m_frameId) {
                        bExist = true;
                        break;
                    }
                }
                if(bExist)
                    continue;
                new_tracks.push_back(track);
            }
            mp->setBad(); //important.
        }
        
        ///update
        new_mp->setBad(false);
        new_mp->removeAllTracks();
        new_mp->addTracks(new_tracks);
       
        //update descriptor.
        std::vector<cv::Mat> descriptors;
        for(const auto& track : new_tracks) {
            ygomi::Frame* frame = m_pGlobalMap->getFrame(track.m_frameId);
            descriptors.push_back(frame->getKeyPoint(track.m_keyPointId).m_descriptor);
        }
        new_mp->mergeDescriptor(descriptors);
        
        // update mean viewing direction and scale invariance distances.
        std::vector<cv::Point3f> cameraCenters;
        std::vector<int> levels;
        const std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                     m_pParamParser->parseInteger("raulLevelNum"));

        int level = -1;
        for (int i = 0; i < new_tracks.size(); i++) {
            ygomi::Frame* mpKF = m_pGlobalMap->getFrame(new_tracks[i].m_frameId);
            const ygomi::KeyPoint& mpKP = mpKF->getKeyPoint(new_tracks[i].m_keyPointId);
            if (mpKF->getType() == ygomi::KEY_FRAME) {
                const cv::Point3f cC = mpKF->getCameraCenter();
                cameraCenters.push_back(cC);
                if (level == -1) {
                    level = mpKP.m_key.octave;
                }
            }
        }
        new_mp->updateNormalAndDepth(cameraCenters, level, scaleFactorSet);
        
        //update first key frame id.
        long nFirstKeyFrameID = new_tracks[0].m_frameId;
        for(const auto& track : new_tracks) {
            ygomi::Frame* frame = m_pGlobalMap->getFrame(track.m_frameId);
            long tempId = frame->getId();
            if(tempId < nFirstKeyFrameID) {
                nFirstKeyFrameID = tempId;
            }
        }
        new_mp->setFirstKeyFrameID(nFirstKeyFrameID);
        //todo: double-check: should set reference kf id?
        
        //update frames.
        for(const auto& track : new_tracks) {
            ygomi::Frame* frame = m_pGlobalMap->getFrame(track.m_frameId);
            //for other frames, we cannot make sure that the observation is unique,
            //so we only add a observation to them.
            frame->addMatchingPair(track.m_keyPointId, new_mp->getId());
        }
        
        //udpate current frame.
        //for current frame, we MUST merge the observation, so reset its observation
        //firstly, then add merged result.
        frame->resetMatchingPair(keyId);
        frame->addMatchingPair(keyId, new_mp->getId());
    }
    
    void Platform::fuseMapPointsKernel(long frameId, const std::vector<long> &mapPointsIndices)
    {
        pDistFunc calcDistfunc;
        float TH_LOW, TH_HIGH;
        float TH_MAX;
        switch (ygomi::KeyPoint::m_descType) {
            case ygomi::BINARY_DESC:
                calcDistfunc = calcDistanceHamming;
                TH_MAX  = 256;
                TH_LOW  = 50;
                TH_HIGH = 100;
                break;
            default:
                calcDistfunc = calcDistanceEuler;
                TH_MAX = 4.0f;
                TH_HIGH = m_pParamParser->parseFloat("fSGDMatchThHigh") * TH_MAX;
                TH_LOW  = m_pParamParser->parseFloat("fSGDMatchThLow") * TH_HIGH;
                break;
        }
        
        const std::vector<float>& invLevelSigma2 = algo::calcScaleSquareSetInv(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                               m_pParamParser->parseInteger("raulLevelNum"));
        const std::vector<float>& scaleFactors   = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                      m_pParamParser->parseFloat("raulLevelNum"));

        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        
#ifdef RUN_TEST
        static int case_index = 0;
        char ch_idx[256];
        memset(ch_idx, 0, sizeof(ch_idx));
        sprintf(ch_idx, "%d", case_index++);
        
        const std::string& case_root = "/Users/zhaiq/Documents/ygomi/project/roadDBCore/SLAM_Matlab_Version/v2.0/unit_test/platform/FuseMapPointKernel/testcase/";
        const std::string& input_kf_path = case_root + "case" + static_cast<std::string>(ch_idx) + "_keyframe_input.txt";
        currframe = readSingleFrame(input_kf_path);
        
        frameId = currframe->getId();
        
        m_pGlobalMap->reset();
        m_pGlobalMap->addFrame(*currframe);
        
        const std::string& input_mp_path = case_root + "case" + static_cast<std::string>(ch_idx) + "_mappoint_input.txt";
        std::vector<ygomi::MapPoint*> allMapPoints = ::readAllMapPoints(input_mp_path);
        
        for(const auto& mp : allMapPoints)
            m_pGlobalMap->addMapPoint(*mp);
#endif
        CV_Assert(currframe);
        
        const auto& keys = currframe->getKeyPoints();
        int nImageWidth  = m_pParamParser->parseInteger("ImageWidth");
        int nImageHeight = m_pParamParser->parseInteger("ImageHeight");
        cv::Rect imgRect(0, 0, nImageWidth, nImageHeight);
        computeImageBounds(imgRect, m_K, m_DistCoef);
        assignFeaturesToGrid(keys);
        
        const cv::Mat& pose3D = currframe->getPose();

        const cv::Point3f& Ow = currframe->getCameraCenter();
#ifdef RUN_TEST
        std::cout << "pose " << pose3D << std::endl;
        std::cout << "camera center: " << Ow << std::endl;
#endif
        const float th = 10.f; //todo: move this to config file.

#ifdef RUN_TEST
        for(size_t id=0; id < allMapPoints.size(); id++) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(allMapPoints[id]->getId());
#else
        for(auto& mpId : mapPointsIndices) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
#endif
            CV_Assert(mp);
            
            if(mp->isBad() || mp->isInKeyFrame(frameId))
                continue;
            
            const cv::Point3f& p3Dw = mp->getPosition();
            
            cv::Point3f p3Dc =  projectWorld2Camera<double>(pose3D, p3Dw);
            
#ifdef RUN_TEST_FUSE
            std::cout << "world pos of mp : " << p3Dw << std::endl;
            std::cout << p3Dc << std::endl;
#endif
            
            if(p3Dc.z < 0.0f)
                continue;
            
            cv::Point2f p2Di = projectCamera2Image<double>(m_K64f, p3Dc);
            
#ifdef RUN_TEST_FUSE
            std::cout << p2Di << std::endl;
#endif
            
            if(!inBoundary(0, 0, nImageWidth, nImageHeight, p2Di.x, p2Di.y))
                continue;
            
            //depth must be inside the scale pyrmid of the image.
            const cv::Point3f& PO = p3Dw - currframe->getCameraCenter();
            float dist3D = cv::norm(PO);
            float fmaxDistance = mp->getMaxDistance();
            float fminDistance = mp->getMinDistance();
            
#ifdef RUN_TEST_FUSE
            std::cout << fmaxDistance << " " << mp->m_maxDistance << std::endl;
            std::cout << fminDistance << " " << mp->m_minDistance << std::endl;
            std::cout << "diff : " << PO << std::endl;
            std::cout << "dist = " << dist3D << std::endl;
#endif
            
            if(dist3D < fminDistance || dist3D > fmaxDistance)
                continue;
            
            //viewing angle must be less than 60 deg.
            const cv::Mat& Pn = mp->getNormal();
#ifdef RUN_TEST_FUSE
            std::cout << Pn << std::endl;
            std::cout << Pn.at<double>(0, 0) << std::endl;
            CV_Assert(Pn.type() == CV_32FC1);
            std::cout << Pn.at<float>(0, 0)*PO.x + Pn.at<float>(1, 0)*PO.y + Pn.at<float>(2, 0)*PO.z << std::endl;
#endif
            //dot-product
            if(Pn.at<float>(0, 0)*PO.x + Pn.at<float>(1, 0)*PO.y + Pn.at<float>(2, 0)*PO.z < 0.5f * dist3D)
                continue;
#ifdef RUN_TEST_FUSE
            std::cout << log(m_pParamParser->parseFloat("raulScaleFactor")) << std::endl;
#endif
            //predict scale.
            int nPredictedLevel = mp->predictScale(dist3D, log(m_pParamParser->parseFloat("raulScaleFactor")), m_pParamParser->parseInteger("raulLevelNum"));
#ifdef RUN_TEST_FUSE            
            std::cout << "predict level : " << nPredictedLevel << std::endl;
#endif
            //search in raduis.
            float radius = th * scaleFactors[nPredictedLevel];
            
            std::vector<size_t> indices;
            std::vector<cv::KeyPoint> kpList;
            kpList.resize(keys.size());
            for (int i = 0; i<kpList.size();i++){
                kpList[i] =keys[i].m_key;
            }
            getFeaturesIndexInArea(kpList, p2Di, radius, indices);
            
#ifdef RUN_TEST_FUSE
            int index = keys.size() / 5;
            std::cout << keys[index].m_key.pt << "  " << keys[index].m_key.octave << std::endl;

            index = keys.size() / 4;
            std::cout << keys[index].m_key.pt << "  " << keys[index].m_key.octave << std::endl;

            index = keys.size() / 3;
            std::cout << keys[index].m_key.pt << "  " << keys[index].m_key.octave << std::endl;

            index = keys.size() / 2;
            std::cout << keys[index].m_key.pt << "  " << keys[index].m_key.octave << std::endl;

            std::cout << "r = " << radius << " u = " << p2Di.x << " v = " << p2Di.y << std::endl;
            std::cout << "candidate size : " << indices.size() << std::endl;
            for(int i=0; i<indices.size(); i++) {
                const auto& kp = keys[indices[i]];
                std::cout << "idx : " << indices[i] << " key : " << kp.m_key.pt << std::endl;
            }
#endif
            if(indices.empty())
                continue;
            
            //match to the most similar key point in the raduis.
            const cv::Mat& mp_desc = mp->getDescriptor();
            CV_Assert(!mp_desc.empty());
            
            float bestDist = TH_MAX;
            int bestIdx    = -1;
            for(auto& keyId : indices) {
                const ygomi::KeyPoint& key = currframe->getKeyPoint(keyId);
                int kpLevel = key.m_key.octave;
                
#ifdef RUN_TEST_FUSE
                std::cout << "kplevel : " << kpLevel << std::endl;
#endif
                
                if(kpLevel < nPredictedLevel-1 || kpLevel> nPredictedLevel)
                    continue;
                
                const cv::Point2f& pt = key.m_key.pt;
                float ex = pt.x - p2Di.x;
                float ey = pt.y - p2Di.y;
                const float e2 = ex * ex + ey * ey;
                
#ifdef RUN_TEST_FUSE
                std::cout << e2 << "----->" << e2 * invLevelSigma2[kpLevel] << std::endl;
#endif
                
                if(e2 * invLevelSigma2[kpLevel]>5.99)
                    continue;
                
                const cv::Mat& key_desc = key.m_descriptor;
                CV_Assert(!key_desc.empty());
                
                float dist = calcDistfunc(mp_desc, key_desc);
                
                if(dist < bestDist) {
                    bestDist = dist;
                    bestIdx  = keyId;
                }
            }
            
            //if there is already a map point replace, oterwise add a new measurement.
            if(bestDist <= TH_LOW) {
                //add new measurment.
                
                AddMpKpConnection(mp->getId(), currframe->getId(), bestIdx);
                
                const ygomi::KeyPoint& key = currframe->getKeyPoint(bestIdx);
                if(key.m_mapPointId.size() > 1) {
                    mergeMapPoints(currframe->getId(), bestIdx);
                }
                std::vector<cv::Mat> descriptors;
                const auto& tracks = mp->getObservation();
                for(const auto& track : tracks) {
                    const ygomi::Frame* frame = m_pGlobalMap->getFrame(track.m_frameId);
                    descriptors.push_back(frame->getKeyPoint(track.m_keyPointId).m_descriptor);
                }
                mp->mergeDescriptor(descriptors);
                
#ifdef RUN_TEST_FUSE
                std::cout << "descriptor : " << mp->getDescriptor() << std::endl;
                std::cout << "id : " << mp->getId() << std::endl;
#endif
            }
        }
    }
    
    void Platform::fuseMapPointsKernel(long frameId, int keyFrameCount)
    {
        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currframe && currframe->getType() == KEY_FRAME);
        
        // get the interest key frames
        const std::vector<ygomi::Frame*>& keyFrames = m_pGlobalMap->getInterestKeyFrames(frameId, keyFrameCount);
        std::vector<ygomi::Frame*> keyFramesTmp = keyFrames;
        keyFramesTmp.insert(keyFramesTmp.end(), currframe); // insert the current frame
        
        for (int i = 0; i < keyFramesTmp.size(); i++)
        {
            Frame* kf = keyFramesTmp[i];
            CV_Assert(kf && kf->getType() == KEY_FRAME);
            
            const std::vector<ygomi::KeyPoint>& keys = kf->getKeyPoints();
            for (int j = 0; j < keys.size(); j++)
            {
                std::vector<long> vecMpId = keys[j].m_mapPointId;
                size_t num = vecMpId.size();
                if ( num > 1)
                {
                    int nTrackLength = 0;
                    int nKeepIdx = -1;
                    for (int n = 0; n < num; n++)
                    {
                        const MapPoint* mp = m_pGlobalMap->getMapPoint(vecMpId[n]);
                        const std::vector<ygomi::Track>& track = mp->getObservation();
                        if (nTrackLength < track.size())
                        {
                            nTrackLength = track.size();
                            nKeepIdx = n;
                        }
                    }
                    MapPoint* mpKeep = m_pGlobalMap->getMapPoint(vecMpId[nKeepIdx]);
                    mpKeep->setBad(false); // set the choosed mp to not bad
                    for(int m=0; m<num; m++) {
                        if(m == nKeepIdx)
                            continue;
                        
                        MapPoint* mp = m_pGlobalMap->getMapPoint(vecMpId[m]);
                        merge2MapPoints(mpKeep, mp);
                        mp->setBad(); // set the deleted mp to bad
                        kf->delMatchingPair(j, vecMpId[m]); // delete one redundant mpId of a certain key point
                    }
                }
            }
        }
    }
    
    void Platform::merge2MapPoints(MapPoint* mpKeep, const MapPoint* mp)
    {
        std::vector<ygomi::Track> trackRef = mpKeep->getObservation();
        std::vector<ygomi::Track> track = mp->getObservation();
        merge2Tracks(trackRef, track);
        mpKeep->removeAllTracks();
        mpKeep->addTracks(trackRef);
    }
    
    bool operator == (const ygomi::Track& track1, const ygomi::Track& track2)
    {
        return track1.m_frameId == track2.m_frameId && track1.m_keyPointId == track2.m_keyPointId;
    }
    
    bool compare(const ygomi::Track& track1, const ygomi::Track& track2)
    {
        return (track1.m_frameId < track2.m_frameId);
    }
    
    void Platform::merge2Tracks(std::vector<ygomi::Track>& trackRef, const std::vector<ygomi::Track>& trackDel)
    {
        for (int i =0; i < trackDel.size(); i++)
        {
            std::vector<ygomi::Track>::iterator iter;
            iter = std::find(trackRef.begin(), trackRef.end(), trackDel[i]);
            if (iter == trackRef.end())
            {
                trackRef.push_back(trackDel[i]);
            }
        }
        std::sort(trackRef.begin(), trackRef.end(), compare);
    }
    
}
