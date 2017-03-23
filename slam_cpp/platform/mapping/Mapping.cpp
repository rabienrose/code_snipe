
/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Mapping.cpp
 * @brief  This should be completed to introduce this document.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.10        Qiang.Zhai     Add localBA and triangulate module.
 *      2016.08.11        Lizao.Zhang    Add searchByEpipolar module.
 *      2016.08.14        Zhongjun.Dai   Add cullMapPoint.
 *******************************************************************************
 */
#include "Platform.hpp"
#include <unordered_map>
#include <cassert>
#include <fstream>

#include "../../../3rdparty/g2o/g2o/core/block_solver.h"
#include "../../../3rdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../../../3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../../../3rdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../../../3rdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "../../../3rdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../../../3rdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include "searchByEpipolarHandle.hpp"
#include "Frame.hpp"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include "TypeDef.hpp"
#include "Platform.hpp"
#include "FeatureTool.hpp"
#include "Utils.h"
#include "TriangulateHandle.hpp"
#include "OptimizeByLocalBA.hpp"
#include "paramParser.hpp"
#include "Utils.h"
#include "NewMPCreator.h"

//#define RUN_TEST

#ifdef RUN_TEST
#   include <fstream>
#   include <gtest/gtest.h>
#endif //RUN_TEST

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

extern void readNextLine(std::ifstream& fin, std::istringstream& istr);

void saveCullKeyFrameCout(int KeyFrameId, int nmp, int nRedundantObservation, const std::string& path);


void saveCullKeyFrameCout(std::vector<int>& Keys, std::vector<int>& nmps, std::vector<int>& nRedundantObservationVec, const std::string& path)
{
    std::ofstream fout;
    fout.open(path.c_str());
    if(!fout.is_open()) {
        return;
    }

    for(size_t i=0; i<nmps.size(); i++){
        if(Keys[i] == 0)
            continue;
        fout <<  "Key Frame= " << Keys[i] << std::endl;
        fout <<  "nmps= "      << nmps[i]  << std::endl;
        fout <<  "nRedundantObservations= " << nRedundantObservationVec[i] << std::endl;
    }

    fout.close();
}


namespace ygomi
{
 
#ifdef RUN_TEST
    void readMapPointsList(const std::string& file_path, std::list<ygomi::MapPoint*>& mapPointLists, int& nCurrentKFId)
    {
        std::ifstream fin;
        fin.open(file_path.c_str());
        
        EXPECT_TRUE(fin.is_open());
        
        std::string str;
        std::istringstream istr;
        
        readNextLine(fin, istr);
        istr >> str;
        istr >> str; nCurrentKFId = atoi(str.c_str());

        readNextLine(fin, istr);
        istr >> str;
        int mp_num;
        istr >> mp_num;
        for(int i=0; i<mp_num; i++)
            mapPointLists.push_back(readSingleMapPoint(fin));
    }
#endif
    
    void Platform::debugTrack(long frameId)
    {
        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        
        if(frame->getType() != ygomi::KEY_FRAME)
            return;
        
        const std::vector<ygomi::KeyPoint> keys = frame->getKeyPoints();
        const std::vector<long> mpIdIndices = frame->getMapPointIndices();
        
        for(int i=0; i<mpIdIndices.size(); i++) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpIdIndices[i]);
            CV_Assert(mp);
            
            const auto& tracks = mp->getObservation();
            bool ok = false;
            for(const auto& track : tracks) {
                if(track.m_frameId == frameId) {
                    ok = true;
                    break;
                }
            }
            if(!ok) {
                int break_point = 1;
            }
        }
    }
    
    void Platform::debugFrame(long frameId)
    {
        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        
        const auto& keys = frame->getKeyPoints();
        for(const auto& key : keys) {
            if(key.m_mapPointId.empty())
                continue;
            
            for(const auto mpId : key.m_mapPointId) {
                const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
                CV_Assert(mp);
                if(cvIsNaN(mp->getPosition().x)) { //un-triangulated.
                    CV_Assert(mp->isBad());
                    const std::vector<ygomi::Track> tracks = mp->getObservation();
                    CV_Assert(tracks.size() == 2);
                }
            }
        }
    }
	
	void Platform::debugMapPoint(long frameId)
	{
		ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
		
		const auto& keys = frame->getKeyPoints();
		for(const auto& key : keys) {
			for(auto id : key.m_mapPointId) {
				ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(id);
				const auto& tracks = mp->getObservation();
				
				for(const auto& track : tracks) {
					CV_Assert(track.m_frameId != frameId);
				}
			}
		}
		
		std::vector<long> mpIndices = frame->getMapPointIndices();
	}
	
    void Platform::map(long frameId)
    {
		ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
		if(!frame || frame->getType() != ygomi::KEY_FRAME) {
            return;
        }
		
		const bool bAutoPause = false;
		//showTrack(frameId, "TrackBeforeMapping", bAutoPause);
		
        int keyFrameCount = m_pParamParser->parseInteger("nCovisibilityNum");

        //debugTrack(frameId);
        //debugFrame(frameId);

        //Check recent map points.
//        cullMapPoint(frameId);
		
        //search matching-pairs by epipolar and try to triangulate.
        //debugTrack(frameId);
        //debugFrame(frameId);
        
        searchByEpipolar(frameId, keyFrameCount);

        //debugTrack(frameId);
        //debugFrame(frameId);
        
        triangulate(frameId, keyFrameCount);

        //Fuse map points duplications.
//        fuseMapPoint(frameId);
        
        //debugTrack(frameId);
        //debugFrame(frameId);
        
        //Local Bandle Adjustment
        if ( /*(frameId < 15 && frameId % 3 ==0) || */frameId % 10 == 0)
        {
			optimizePoseByLocalBA(frameId);
        }
        
        //Check radundant local key frames.
//        cullKeyFrame(frameId);
		
        //debugTrack(frameId);
        //debugFrame(frameId);
//        kf_center_dis2_->append(computeCenterDis2_withPrev(frameId));
//
    }
    
    void Platform::createNewMapPoints(long frameId, int keyFrameCount)
    {
        Frame* pFrame = m_pGlobalMap->getFrame(frameId);
        const std::vector<ygomi::KeyPoint>& refKps = pFrame->getKeyPoints();
        std::vector<cv::KeyPoint> refKpsCv;
        refKpsCv.resize(refKps.size());
        std::vector<cv::Mat> refDescs;
        refDescs.resize(refKps.size());
        std::vector<bool> refMpMask;
        refMpMask.resize(refKps.size());
        for (int i=0; i<refKps.size(); i++){
            refKpsCv[i] = refKps[i].m_key;
            refDescs[i] = refKps[i].m_descriptor;
            if (refKps[i].m_mapPointId.size()==0){
                refMpMask[i] = true;
            }else{
                refMpMask[i] = false;
            }
        }
        std::vector<float> scaleFactors = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                      m_pParamParser->parseFloat("raulLevelNum"));
        float scaleFactor = m_pParamParser->parseFloat("raulScaleFactor");
        cv::Mat obPose = pFrame->getPose();
        algo::NewCloudPointCreator cp_ceator(obPose, m_K, scaleFactor, 0, scaleFactors, refKpsCv, refDescs, refMpMask);
        
        std::vector<ygomi::Frame*> pframes = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, keyFrameCount);
        
        for(std::size_t i =0 ; i< pframes.size(); i++)
        {

            if(isKeyFrameTooClose(frameId, pframes[i]->getId()))
                continue;
            
            const std::vector<ygomi::KeyPoint>& obKps = pframes[i]->getKeyPoints();
            std::vector<cv::KeyPoint> obKpsCv;
            obKpsCv.resize(obKps.size());
            std::vector<cv::Mat> obDescs;
            obDescs.resize(obKps.size());
            std::vector<bool> obMpMask;
            obMpMask.resize(obKps.size());
            for (int i=0; i<obKps.size(); i++){
                obKpsCv[i] = obKps[i].m_key;
                obDescs[i] = obKps[i].m_descriptor;
                if (obKps[i].m_mapPointId.size()==0){
                    obMpMask[i] = true;
                }else{
                    obMpMask[i] = false;
                }
            }
            cp_ceator.set(pframes[i]->getPose(), obKpsCv, obDescs, obMpMask );
            std::vector<std::pair<size_t, size_t> > matchedPairs;
            std::vector<cv::Point3f> posiList;
            cp_ceator(matchedPairs, posiList);
            
            fill2DMatches(pframes[i]->getId(), pFrame->getId(), matchedPairs, posiList);
        }	
    }
    
    bool Platform::isKeyFrameTooClose(long referFrameId, long normalFrameId)
    {
        
        const ygomi::Frame* referFrame = m_pGlobalMap->getFrame(referFrameId);
        CV_Assert(referFrame);
        const cv::Point3f& center1 = referFrame->getCameraCenter();
        
        
        const ygomi::Frame* normalFrame = m_pGlobalMap->getFrame(normalFrameId);
        CV_Assert(normalFrame);
        const cv::Point3f& center2 = normalFrame->getCameraCenter();
        
        float diff = cv::norm(center1 - center2);
        float medianDepth = m_pGlobalMap->computeNthDepth(normalFrameId, 2);
        
        float r = diff / medianDepth;
        float fTooCloseTh = m_pParamParser->parseFloat("fCheckIsKeyFrameTooCloseTh");
        
        return r < fTooCloseTh;
    }
    
    void Platform::searchByEpipolar(long frameId, int keyFrameCount)
    {
        //debugTrack(frameId);
        
        //get current key frame.
        ygomi::Frame* referKeyframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(referKeyframe);
        FrameInfo referUnmatched;
        const std::vector<ygomi::KeyPoint>& referKeys = referKeyframe->getKeyPoints();
        if(referKeys.empty())
            return;
        const size_t keyNum = referKeys.size();
        const int descriptorDim  = referKeys[0].m_descriptor.cols;
        const int descriptorType = referKeys[0].m_descriptor.type();
        referUnmatched.descriptors.create(keyNum, descriptorDim, descriptorType);
        std::vector<size_t> referIndices;
        int count = 0;
        for(size_t id=0; id<keyNum; id++) {
            if(referKeys[id].m_mapPointId.empty()){
                referUnmatched.keys.push_back(referKeys[id].m_key);
                referKeys[id].m_descriptor.copyTo(referUnmatched.descriptors.row(count++));
                referIndices.push_back(id);
            }
        }
        cv::Mat referPose =referKeyframe->getPose();
        referPose.convertTo(referPose, CV_32FC1);
        referUnmatched.pose = referPose;

        //parse input parameters.
        std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                               m_pParamParser->parseInteger("raulLevelNum"));

        //get reference key frames set.
        const std::vector<ygomi::Frame*> keyframes = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, keyFrameCount);
        for(size_t id=0; id<keyframes.size(); id++) {
            const ygomi::Frame* neighkf = keyframes[id];
            
            if(isKeyFrameTooClose(referKeyframe->getId(), neighkf->getId()))
				;//continue;
			
            FrameInfo neighUnmatched;
            const std::vector<ygomi::KeyPoint>& neighKeys = neighkf->getKeyPoints();
            size_t keyNum2 = neighKeys.size();
            std::vector<size_t> neighIndices;
            neighUnmatched.descriptors.create(keyNum2, descriptorDim, descriptorType);
            count = 0;
            for(size_t id=0; id<keyNum2; id++){
            // check if refer 2D point have mapping 3D point
                if(neighKeys[id].m_mapPointId.empty()){
                    neighUnmatched.keys.push_back(neighKeys[id].m_key);
                    neighKeys[id].m_descriptor.copyTo(neighUnmatched.descriptors.row(count++));
                    neighIndices.push_back(id);
                }
            }
            cv::Mat neighPose =neighkf->getPose();
            neighPose.convertTo(neighPose, CV_32FC1);
            neighUnmatched.pose = neighPose;
			
			//calculate median depth
			std::vector<ygomi::KeyPoint> kps1 = neighkf->getKeyPoints();
			std::vector<cv::Point3f> mpPosiSet;
			for (int i = 0; i < kps1.size(); i++) {
				std::vector<long> mpId = kps1[i].m_mapPointId;
				if (mpId.empty()) {
					continue;
				}
				MapPoint* mp = m_pGlobalMap->getMapPoint(mpId[0]);
				mpPosiSet.push_back(mp->getPosition());
			}
			float medianDepth = neighkf->computeSceneMedianDepth(mpPosiSet, 2);
			neighUnmatched.medianDepth = medianDepth;
//            float TH_MAX = 4.0f;
            TH_HIGH = m_pParamParser->parseFloat("fSGDMatchThHigh") * TH_MAX;
            TH_LOW  = m_pParamParser->parseFloat("fSGDMatchThLow") * TH_HIGH;
            // do searchByEpipolar
            std::vector<std::pair<size_t, size_t> > matchedPairs;
            std::vector<int8_t> referKeyMatchedFlag;
            std::vector<int8_t> neighKeyMatchedFlag;
            searchByEpipolarCore(
                                 referUnmatched,
                                 neighUnmatched,
                                 m_K,
                                 scaleFactorSet,
                                 TH_LOW,
                                 matchedPairs,
                                 referKeyMatchedFlag,
                                 neighKeyMatchedFlag);
        
            //update the GlobalMap
            for(auto& pair : matchedPairs) {
                pair.first  = referIndices[pair.first];
                pair.second = neighIndices[pair.second];
            }

            //debugTrack(frameId);
            
            std::vector<cv::Point3f> posi;
            fill2DMatches(referKeyframe->getId(), neighkf->getId(), matchedPairs, posi);

//            showTrack(frameId, "after each searchByEpipolarCore", false);
            //debugTrack(frameId);
		}
    }
    
    void Platform::triangulate(long frameId, int keyFrameCount)
    {
        Frame* currFrame        = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currFrame);
        const cv::Mat& currPose = currFrame->getPose();
        
        std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                               m_pParamParser->parseInteger("raulLevelNum"));

        const float scaleFactor = m_pParamParser->parseFloat("raulScaleFactor");

        std::vector<cv::KeyPoint> currKeys;
        std::vector<cv::KeyPoint> referKeys;
        std::vector<long> mapPointdIndices;
        std::vector<std::pair<size_t, size_t> > matchingPairs; //current-reference
        std::vector<cv::Mat> mapPointsPosition;
        std::vector<int> status;
        const std::vector<ygomi::Frame*> keyframes = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, keyFrameCount);
		
		std::vector<long> mpIndices;
		
        for(const auto& keyframe : keyframes) {
            currKeys.clear();
            referKeys.clear();
            mapPointdIndices.clear();
            mapPointsPosition.clear();
            matchingPairs.clear();
            status.clear();
            
            CV_Assert(keyframe);
            const cv::Mat& referPose = keyframe->getPose();
            
            //find current and reference un-triangulated keys.
            for(const auto& key : currFrame->getKeyPoints()) {
    
                if(key.m_mapPointId.empty()) continue;
    
                for(auto mpId : key.m_mapPointId) {
                    const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
                    CV_Assert(mp);
//                    if(mp->isBad()) { //un-triangulated.
                    if(cvIsNaN(mp->getPosition().x)) {
                        const std::vector<ygomi::Track> tracks = mp->getObservation();
                        CV_Assert(tracks.size() == 2);
                        int currOrder = -1;//  = tracks[0].m_frameId == currFrame->getId() ? 0 : 1;
                        int referOrder = -1;// = currOrder == 0 ? 1 : 0;
                        for (int i=0; i<tracks.size();i++){
                            if (tracks[i].m_frameId == keyframe->getId()){
                                referOrder = i;
                            }
                            if (tracks[i].m_frameId == currFrame->getId()){
                                currOrder = i;
                            }
                        }
                        if (currOrder == -1){
                            std::cout<<"triangulate: track no current frame!!"<<std::endl;
                            CV_Assert(currOrder == -1);
                        }
                        if(referOrder == -1){
                            continue;
                        }
                        currKeys.push_back(currFrame->getKeyPoints()[tracks[currOrder].m_keyPointId].m_key);
                        referKeys.push_back(keyframe->getKeyPoints()[tracks[referOrder].m_keyPointId].m_key);
                        mapPointdIndices.push_back(mp->getId());
                        matchingPairs.push_back(std::make_pair(tracks[currOrder].m_keyPointId, tracks[referOrder].m_keyPointId));
                    }
                }
            }

            //triangulate.
            ::triangulate(currKeys,          //2D keys in current frame.
                          currPose,          //camera pose of current frame.
                          referKeys,         //2D keys in refer frame.
                          referPose,         //camera pose of refer frame.
                          m_K,               //camera parameters.
                          scaleFactor,       //scale factor
                          mapPointsPosition, //returned map point position.
                          status);           //triangulated status for each pair.


            //update.
            size_t count = 0;

            for(size_t id=0; id<status.size(); id++) {
                ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mapPointdIndices[id]);
                if(status[id]) { //trangulate success, then update the map point.
                    const cv::Mat& position = mapPointsPosition[count++];
                    cv::Point3f pt(position.at<float>(0, 0), position.at<float>(0, 1), position.at<float>(0, 2));
                    
                    mp->setPosition(pt);
                    mp->setBad(false); //valid map points.
					//==================== update Normal and Depth =======================
                    const std::vector<ygomi::Track>& obserations = mp->getObservation();
                    if (!obserations.empty()) {
                        std::vector<cv::Point3f> cameraCenters;
                        std::vector<int> levels;
                        int referId = mp->getReferenceKeyFrameID();
                        int level = -1;
                        for (int i = 0; i < obserations.size(); i++) {
                            ygomi::Frame* mpKF = m_pGlobalMap->getFrame(obserations[i].m_frameId);
                            const ygomi::KeyPoint& mpKP = mpKF->getKeyPoint(obserations[i].m_keyPointId);
                            if (mpKF->getType() == ygomi::KEY_FRAME) {
                                const cv::Point3f cC = mpKF->getCameraCenter();
                                cameraCenters.push_back(cC);
                                if (mpKF->getId() == referId) {
                                    level = mpKP.m_key.octave;
                                }
                            }
                        }
                        mp->updateNormalAndDepth(cameraCenters, level, scaleFactorSet);
                    }
                    //===================================================================
					mpIndices.push_back(mp->getId());
                }
                else { //trigulate failed, remove connections.
                    DelMpKpConnection(mp->getId(),  currFrame->getId(), matchingPairs[id].first);
                    DelMpKpConnection(mp->getId(),  keyframe->getId(), matchingPairs[id].second);
                }
            }
        }
		//showTrack(frameId, mpIndices, "new map points", false);
    }
    
    void Platform::cullMapPoint(long frameId)
    {
        int nThObs              = m_pParamParser->parseInteger("nMapPointCullingThObs");
        float foundRatioTh      = m_pParamParser->parseFloat("fMapPointCullingFoundRatioTh");
        
        const int cnThObs       = nThObs;
//        const int nCurrentKFId  = static_cast<int>(frameId);
        int nCurrentKFId  = static_cast<int>(frameId);
        
        std::list<ygomi::MapPoint*> mapPointLists;
        const auto& mapPointsIndices = m_pGlobalMap->getRecentAddedMapPointsIndices();
        for(const auto& mpId : mapPointsIndices) {
            mapPointLists.push_back(m_pGlobalMap->getMapPoint(mpId));
        }
#ifdef RUN_TEST
        mapPointLists.clear();
        
        static int case_index = 0;
        char ch_idx[256];
        memset(ch_idx, 0, sizeof(ch_idx));
        sprintf(ch_idx, "%d", case_index++);
        
        const std::string& input_file_name = "/Users/zhaiq/Desktop/SLAM2.0Test/MapPointCulling_Case" + static_cast<std::string>(ch_idx) + "_input.txt";
        readMapPointsList(input_file_name, mapPointLists, nCurrentKFId);
        
        std::list<ygomi::MapPoint*> copy_MapPoints = mapPointLists;
#endif //RUN_TEST
        
        std::list<ygomi::MapPoint*>::iterator lit = mapPointLists.begin();
        
        while (lit != mapPointLists.end()) {
            ygomi::MapPoint* mp = *lit;
            if(!mp) {
                lit = mapPointLists.erase(lit);
                continue;
            }
            
            if(mp->isBad()) {
                lit = mapPointLists.erase(lit);
            }
            else if(mp->getFoundRatio() < foundRatioTh) {
                mp->setBad(true);
                lit = mapPointLists.erase(lit);
            }
            else if((nCurrentKFId - mp->getFirstKeyFrameID()) >= 2 && mp->getObservation().size() < cnThObs) {
                mp->setBad(true);
                lit = mapPointLists.erase(lit);
            }
            else if(nCurrentKFId - mp->getFirstKeyFrameID() >= 3) {
                lit = mapPointLists.erase(lit);
            }
            else
                lit++;
        }
        
#ifdef RUN_TEST
        const std::string& verify_file_path = "/Users/zhaiq/Desktop/SLAM2.0Test/MapPointCulling_Case" + static_cast<std::string>(ch_idx) + "_output.txt";
        std::list<ygomi::MapPoint*> verifyMapPointsList;
        int verifyCurrentKFId;
        readMapPointsList(verify_file_path, verifyMapPointsList, verifyCurrentKFId);
        
        EXPECT_EQ(verifyCurrentKFId, nCurrentKFId);
        EXPECT_EQ(verifyMapPointsList.size(), copy_MapPoints.size());
        
        std::list<ygomi::MapPoint*>::iterator iter        = copy_MapPoints.begin();
        std::list<ygomi::MapPoint*>::iterator verify_iter = verifyMapPointsList.begin();
        const auto& end_iter        = copy_MapPoints.end();
        const auto& verify_end_iter = verifyMapPointsList.end();
        
        for(; iter != end_iter && verify_iter != verify_end_iter; iter++, verify_iter++) {
            const ygomi::MapPoint* mp        = *iter;
            const ygomi::MapPoint* verify_mp = *verify_iter;
            
            EXPECT_EQ(verify_mp->getId(), mp->getId());
            EXPECT_EQ(verify_mp->isBad(), mp->isBad());
            //.....
        }
#endif //RUN_TEST
    }
    
    static inline void readNextLine(std::ifstream& fin, std::istringstream& istr)
    {
        std::string line;
        std::getline(fin, line);
        istr.clear();
        istr.str(line);
    }
    
#ifdef RUN_TEST
    void readAllKeyFrames(const std::string& file_path, long& frameId, std::vector<ygomi::Frame*>& allKeyFrames)
    {
        
        std::ifstream fin;
        fin.open(file_path.c_str());
        if(!fin.is_open())
            throw "cannot open source file.\n";
        
        std::string  str;
        std::istringstream istr;
        
        readNextLine(fin, istr);
        istr >> str;
        istr >> str;    frameId = atol(str.c_str());
        
        readNextLine(fin, istr);
        istr >> str;
        long frameCount;
        istr >> str;    frameCount = atol(str.c_str());
        
        for(long id=0; id<frameCount; id++) {
            allKeyFrames.push_back(readSingleFrame(fin));
        }
    }

    void Platform::readAllMapPoints(const std::string& file_path, std::vector<ygomi::MapPoint*>& allMapPoints)
    {
        std::ifstream fin;
        fin.open(file_path.c_str());
        if(!fin.is_open()) {
            std::cout << "cannot open source file\n";
            return;
        }
        
        std::string line, str;
        std::istringstream istr;
        
        readNextLine(fin, istr);
        istr >> str;
        long mpCount;
        istr >> str;    mpCount = atol(str.c_str());
        
        for(long id=0; id<mpCount; id++) {
            allMapPoints.push_back(readSingleMapPoint(fin));
        }
        
        fin.close();
    }
    
    void writeAllKeyFrames(const std::string& file_path, long frameId, const std::vector<ygomi::Frame*>& allKeyFrames)
    {
        std::ofstream fout;
        fout.open(file_path.c_str());
        if(!fout.is_open())
            throw "";
        
        fout << "param_kf_id= " << frameId << std::endl;
        fout << "kf_size= " << allKeyFrames.size() << std::endl;
        for(const auto& keyframe : allKeyFrames) {
            fout << "id= " << keyframe->getId() << std::endl;
            fout << "refId= " << keyframe->getRefFrameId() << std::endl;
            fout << "bad= " << (keyframe->getType() == ygomi::BAD_KEY_FRAME ? 1 : 0) << std::endl;
            fout << "pose= " << std::endl;
            const cv::Mat& pose = keyframe->getPose();
            for(int i=0; i<3; i++) {
                for(int j=0; j<4; j++) {
                    fout << pose.at<double>(i, j) << " ";
                }
                fout << std::endl;
            }
            fout << "0 0 0 1 " << std::endl;
            fout << std::endl;
            fout << "camera_center= " << keyframe->getCameraCenter().x << " "
                                      << keyframe->getCameraCenter().y << " "
                                      << keyframe->getCameraCenter().z << std::endl;
            const auto& keys = keyframe->getKeyPoints();
            fout << "key_size= " << keys.size() << std::endl;
            for(const auto& key : keys) {
                fout << key.m_key.pt.x << " " << key.m_key.pt.y << " " << key.m_key.octave << " " << key.m_key.class_id << std::endl;
                const cv::Mat& desc = key.m_descriptor;
                for(int i=0; i<desc.cols; i++) {
                    fout << desc.at<float>(0, i) << " ";
                }
                fout << std::endl;
                
                fout << "mp_list_num= " << key.m_mapPointId.size() << std::endl;
                for(auto& mpId : key.m_mapPointId) {
                    fout << mpId << std::endl;
                }
            }
        }
        
        fout.close();
    }
    
    void writeAllMapPoints(const std::string& file_path, const std::vector<ygomi::MapPoint*>& allMapPoints)
    {
        std::ofstream fout;
        fout.open(file_path.c_str());
        if(!fout.is_open())
            throw "";
        
        fout << "mp_size= " << allMapPoints.size() << std::endl;
        for(const auto& mp : allMapPoints) {
            fout << "is_bad= " << mp->isBad() << std::endl;
            fout << "id= " << mp->getId() << std::endl;
            const cv::Point3f& pt = mp->getPosition();
            fout << "pt= " << pt.x << " " << pt.y << " " << pt.z << std::endl;
            
            fout << "desc= " << std::endl;
            const cv::Mat& desc = mp->getDescriptor();
            for(int i=0; i<desc.cols; i++) {
                fout << desc.at<float>(0, i) << " ";
            }
            fout << std::endl;
            
            const auto& tracks = mp->getObservation();
            fout << "track_size= " << tracks.size() << std::endl;
            for(const auto& track : tracks) {
                fout << track.m_frameId << " " << track.m_keyPointId << std::endl;
            }
            
            fout << "first_kf_id= " << mp->getFirstKeyFrameID() << std::endl;
            fout << "min_dist= " << mp->getMinDistance() << std::endl;
            fout << "max_dist= " << mp->getMaxDistance() << std::endl;
            fout << "nvisible= " << mp->getVisible() << std::endl;
            fout << "nrealfoud= " <<mp->getRealFould() << std::endl;
        }
        
        fout.close();
    }
    
#endif //RUN_TEST
    
    void Platform::fuseMapPoint(long frameId)
    {
#ifdef RUN_TEST
        static int case_index = 5;
        char ch_idx[256];
        memset(ch_idx, 0, sizeof(ch_idx));
        sprintf(ch_idx, "%d", case_index++);
        
        //read all key frames.
        const std::string& input_kf_file_name = "/Users/zhaiq/Desktop/SLAM2.0Test/fuseMapPoint/AllKeyFrames_Case" + static_cast<std::string>(ch_idx) + "_input.txt";
        std::vector<ygomi::Frame*> allKeyFrames;
        readAllKeyFrames(input_kf_file_name, frameId, allKeyFrames);
        
        //read all map points.
        const std::string& input_mp_file_name = "/Users/zhaiq/Desktop/SLAM2.0Test/fuseMapPoint/AllMapPoints_Case" + static_cast<std::string>(ch_idx) + "_input.txt";
        std::vector<ygomi::MapPoint*> allMapPoints;
        readAllMapPoints(input_mp_file_name, allMapPoints);
        
        const std::string& output_kf_file_name = "/Users/zhaiq/Desktop/SLAM2.0Test/fuseMapPoint/AllKeyFrames_Case" + static_cast<std::string>(ch_idx) + "_output.txt";
        writeAllKeyFrames(output_kf_file_name, frameId, allKeyFrames);
        
        const std::string& output_mp_file_name = "/Users/zhaiq/Desktop/SLAM2.0Test/fuseMapPoint/AllMapPoints_Case" + static_cast<std::string>(ch_idx) + "_output.txt";
        writeAllMapPoints(output_mp_file_name, allMapPoints);
        
        m_pGlobalMap->reset();
        for(auto& frame : allKeyFrames) {
            m_pGlobalMap->addFrame(*frame);
            std::cout << frame->getId() << "    " << frame->getCameraCenter() << std::endl;
        }
        
        const auto& keyframes = m_pGlobalMap->getAllFrames();
        for(const auto& frame : keyframes) {
            std::cout << frame->getId() << "    " << frame->getCameraCenter() << std::endl;
        }
        
        const std::vector<float> scaleFactorSet_1 = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                     m_pParamParser->parseInteger("raulLevelNum"));
        for(auto& mp : allMapPoints) {
            std::vector<cv::Point3f> cameraCenters;
            const auto& tracks = mp->getObservation();
            if(tracks.empty())
                continue;
            
            const ygomi::Frame* pRefKF = m_pGlobalMap->getFrame(tracks[0].m_frameId);
            int level = pRefKF->getKeyPoint(tracks[tracks.size()-1].m_keyPointId).m_key.octave;
            
            for (int i = 0; i < tracks.size(); i++) {
                const ygomi::Frame* frame = m_pGlobalMap->getFrame(tracks[i].m_frameId);
                CV_Assert(frame);
                
                std::cout << frame->getId() << "    " << frame->getCameraCenter() << std::endl;
                
                if (frame->getType() == ygomi::KEY_FRAME) {
                    const cv::Point3f& cC = frame->getCameraCenter();
                    cameraCenters.push_back(cC);
                }
            }
            mp->updateNormalAndDepth(cameraCenters, level, scaleFactorSet_1);
            
            m_pGlobalMap->addMapPoint(*mp);
        }
        m_pGlobalMap->updateAllKeyFrames();
        m_pGlobalMap->updateCovisibilityGraph();
        

        
#endif //RUN_TEST
        
        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currframe && currframe->getType() == KEY_FRAME);
        
        // Get neighbours directly connect to current key frames.
        int nDirectNeighKFNum   = m_pParamParser->parseInteger("nSearchInNeighborsFirstNeighborsTh");
        const std::vector<ygomi::Frame*>& pDirectNeighKeyFrames = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, nDirectNeighKFNum);
        
        //verify covibisility graph.
#ifdef RUN_TEST
//        std::vector<int> indices = {250, 775, 853, 2257, 1617, 1689, 2134, 401, 91, 436};
//        std::vector<int> indices = {1118, 2060, 2058, 799, 2477, 1158, 914, 108, 1198, 696};
//        for(const auto& neigh : pDirectNeighKeyFrames) {
//            std::cout << "frame id : " << neigh->getId() << "\n";
//            const auto& keys = neigh->getKeyPoints();
//            
//            std::cout << "keys count : " << keys.size() << "\n";
//            
//            for(auto& index : indices) {
//                std::cout << "id : " << index << " ";
//                std::cout << keys[index].m_key.pt << std::endl;
//                if(!keys[index].m_mapPointId.empty()) {
//                    std::cout << "connected mp id : " << keys[index].m_mapPointId[0] << std::endl;
//                }
//                else {
//                    std::cout << "empty connections\n";
//                }
//            }
//        }
#endif //RUN_TEST
        
        // Search keyframes in covisibility graph.
        int nIndirectNeighKFNum = m_pParamParser->parseInteger("nSearchInNeighborsSecondNeighborsTh");
        std::vector<ygomi::Frame*> pTargetKeyFrames;
        for(const auto& keyframe : pDirectNeighKeyFrames) {
            bool bExist = std::find(pTargetKeyFrames.begin(), pTargetKeyFrames.end(), keyframe) != pTargetKeyFrames.end();
            if(!keyframe || keyframe->getType() == ygomi::BAD_KEY_FRAME || bExist)
                continue;
            
            pTargetKeyFrames.push_back(keyframe);
            
            const std::vector<ygomi::Frame*> pIndirectNeighKeyFrames = m_pGlobalMap->getBestCovisibilityKeyFrames(keyframe->getId(), nIndirectNeighKFNum);
            for(const auto& keyframe2 : pIndirectNeighKeyFrames) {
                bool bExist           = std::find(pTargetKeyFrames.begin(), pTargetKeyFrames.end(), keyframe2) != pTargetKeyFrames.end();
                bool bSameWithCurrent = currframe == keyframe2;
                if(!keyframe2 || keyframe2->getType() == ygomi::BAD_KEY_FRAME || bExist || bSameWithCurrent)
                    continue;
                
                pTargetKeyFrames.push_back(keyframe2);
            }
        }
        
        ///projection back.
        //Check map points cannot observe keyframe.
        //todo: In vehicle SLAM, only project map points that constructed just now.
        
        std::vector<ygomi::MapPoint*> currNewMapPoints;
        currNewMapPoints.reserve(m_newMapPointId.size());
        for(auto& mpId : m_newMapPointId) {
            MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
            if(mp)
                currNewMapPoints.push_back(mp);
        }
        
        if(currNewMapPoints.empty())
            return;
        
        //parse paramters for fusing.
        const cv::Rect imgRect(0, 0,
                               m_pParamParser->parseInteger("ImageWidth"),
                               m_pParamParser->parseInteger("ImageHeight"));
//        int radius          = m_pParamParser->parseInteger("searchRadius");
//        int distThreshold   = m_pParamParser->parseInteger("distThreshold");
        
        //search matches by projection from current KF in target KFs.
        for(auto& keyframe : pTargetKeyFrames) {
            CV_Assert(keyframe && keyframe->getType() == ygomi::KEY_FRAME);
            fuseMapPointsKernel(keyframe->getId(), m_newMapPointId);
        }
        
        //search matches by projection from target KFs in current KF.
        std::vector<long> pFuseCandidateMPIndice;
        pFuseCandidateMPIndice.reserve(pTargetKeyFrames.size() * m_newMapPointId.size());
        
        for(const auto& keyfame : pTargetKeyFrames) {
            const auto& mpIndices = keyfame->getMapPointIndices();
            for(auto& mpId : mpIndices) {
                bool bExist = std::find(pFuseCandidateMPIndice.begin(), pFuseCandidateMPIndice.end(), mpId) != pFuseCandidateMPIndice.end();
                if(bExist)
                    continue;
                pFuseCandidateMPIndice.push_back(mpId);
            }
        }
        
        fuseMapPointsKernel(frameId, pFuseCandidateMPIndice);

        float scaleFactor = m_pParamParser->parseFloat("raulScaleFactor");
        int   octaveNum   = m_pParamParser->parseInteger("raulLevelNum");
        const std::vector<float> scaleFactorSet = algo::calcScaleSet(scaleFactor, octaveNum); //parse input parameters.

        //update points.
        const auto& mapPointIndices = currframe->getMapPointIndices();
        for(auto& mpId : mapPointIndices) {
            ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
            
            if(!mp || mp->isBad())
                continue;
            
            const auto& tracks = mp->getObservation();
            std::vector<cv::Mat> descriptors;
            std::vector<cv::Point3f> KFCameraCenters;
            descriptors.reserve(tracks.size());
            KFCameraCenters.reserve(tracks.size());
            int nlevel = -1;
            for(const auto& track : tracks) {
                const ygomi::Frame* frame  = m_pGlobalMap->getFrame(track.m_frameId);
                const ygomi::KeyPoint& key = frame->getKeyPoint(track.m_keyPointId);
                descriptors.push_back(key.m_descriptor);
                KFCameraCenters.push_back(frame->getCameraCenter());
                if(-1 == nlevel) {
                    nlevel = key.m_key.octave;
                }
            }
            
            //
            mp->updateNormalAndDepth(KFCameraCenters, nlevel, scaleFactorSet); //nlevel, re-calculate, bug.

            //update descriptors.
            mp->mergeDescriptor(descriptors);
        }
        
        //update covisibility gragh.
        m_pGlobalMap->updateCovisibilityGraph(frameId);
    }
	
	void Platform::optimizePoseByLocalBA(long frameId)
	{
		std::ofstream fout("com_vertex.txt");
		
		fout << "current kf id : " << frameId << "\n";
		
		ygomi::Frame* referKF = m_pGlobalMap->getFrame(frameId);
		CV_Assert(referKF && referKF->getType() == ygomi::KEY_FRAME);
		
		std::list<long> localKFIndices;
		localKFIndices.push_back(referKF->getId());
		
		//all neighbor key frames.
		const std::vector<ygomi::Frame*> neighKFs = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, -1);
		for(const auto& kf : neighKFs) {
			if( kf ) {
				localKFIndices.push_back(kf->getId());
			}
		}
		
		std::list<long> localMPIndices;
		for(const auto& kfId : localKFIndices) {
			const auto& kf = m_pGlobalMap->getFrame(kfId);
			const auto& mpIndices = kf->getMapPointIndices();
			for(auto& mpId : mpIndices) {
				ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
				if( mp && !mp->isBad() ) {
					if( std::find(localMPIndices.begin(), localMPIndices.end(), mp->getId()) == localMPIndices.end() ) {
						localMPIndices.push_back(mp->getId());
					}
				}
			}
		}
		
		std::list<long> fixedCameraKFIndices;
		for(const auto& mpId : localMPIndices) {
			const auto& mp = m_pGlobalMap->getMapPoint(mpId);
			const auto& observations = mp->getObservation();
			for(auto& ob : observations) {
				ygomi::Frame* kf = m_pGlobalMap->getFrame(ob.m_frameId); //may be a bug.
				if( kf &&
				   std::find(localKFIndices.begin(), localKFIndices.end(), kf->getId()) == localKFIndices.end() &&
				   std::find(fixedCameraKFIndices.begin(), fixedCameraKFIndices.end(), kf->getId()) == fixedCameraKFIndices.end())
				{
					fixedCameraKFIndices.push_back(kf->getId());
				}
			}
		}
		
		
		fout << "neighs : \n";
		for(auto& neigh : neighKFs) {
			fout << neigh->getId() << " ";
		}
		fout << "\n";
		fout << "local kf indices : \n";
		for(auto& kfId : localKFIndices) {
			fout << kfId << " ";
		}
		fout << "\n";
		
		fout << "local mp indices : \n";
		int indices = 0;
		for(auto& mpId : localMPIndices) {
			fout << mpId << " ";
			if(indices++ % 30 == 0)
				fout << "\n";
		}
		fout << "\n";
		
		fout << "fixed camera indice : \n";
		for(auto& kfId : fixedCameraKFIndices) {
			fout << kfId << " ";
		}
		fout << "\n";
		
		//read kfs and mps.
		std::list<ygomi::Frame*> lLocalKeyFrames, lFixedCameras;
		for(auto& kfId : localKFIndices) {
			ygomi::Frame* kf = m_pGlobalMap->getFrame(kfId);
			lLocalKeyFrames.push_back(kf);
		}
		for(auto& kfId : fixedCameraKFIndices) {
			ygomi::Frame* kf = m_pGlobalMap->getFrame(kfId);
			lFixedCameras.push_back(kf);
		}
		
		std::list<ygomi::MapPoint*> lLocalMapPoints;
		for(auto& mpId : localMPIndices) {
			ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
			lLocalMapPoints.push_back(mp);
		}

		//set up optimizer.g2o::SparseOptimizer optimizer;
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
		
		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
		
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);
		
		bool pbStopFlag = false;
		if( &pbStopFlag ) {
			optimizer.setForceStopFlag(&pbStopFlag);
		}
		
		unsigned long maxKFid = 0;
		std::vector<int> vertexId;
		// Set Local KeyFrame vertices
		for(std::list<ygomi::Frame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
		{
			const ygomi::Frame* pKFi = *lit;
			if(!pKFi) continue;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(algo::toSE3Quat(pKFi->getPose()));
			vSE3->setId(pKFi->getId());
			vSE3->setFixed(pKFi->getId() == 0); //set the first frame as reference.
			
			fout << pKFi->getPose() << "\n" << pKFi->getId() << "\n";
			
			optimizer.addVertex(vSE3);
			if(pKFi->getId()>maxKFid)
				maxKFid=pKFi->getId();
			
			vertexId.push_back(pKFi->getId());
		}
		
		// Set Fixed KeyFrame vertices
		for(std::list<ygomi::Frame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
		{
			const ygomi::Frame* pKFi = *lit;
			if(!pKFi) continue;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(algo::toSE3Quat(pKFi->getPose()));
			vSE3->setId(pKFi->getId());
			vSE3->setFixed(true);
			
			fout << pKFi->getPose() << "\n" << pKFi->getId() << "\n";
			
			optimizer.addVertex(vSE3);
			if(pKFi->getId()>maxKFid)
				maxKFid=pKFi->getId();
			
			vertexId.push_back(pKFi->getId());
		}
		
		// Set MapPoint vertices
		const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
		
		std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
		vpEdgesMono.reserve(nExpectedSize);
		
		std::vector<ygomi::Frame*> vpEdgeKFMono;
		vpEdgeKFMono.reserve(nExpectedSize);
		
		std::vector<ygomi::MapPoint*> vpMapPointEdgeMono;
		vpMapPointEdgeMono.reserve(nExpectedSize);
		
		std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
		vpEdgesStereo.reserve(nExpectedSize);
		
		const float thHuberMono = sqrt(5.991);
		
		std::vector<float> invScaleFactorSigma2 = algo::calcScaleSquareSetInv(m_pParamParser->parseFloat("raulScaleFactor"),
																			  m_pParamParser->parseInteger("raulLevelNum"));
		
		fout << "mp start*******start********\n";
		
		std::list<ygomi::MapPoint*>::iterator lit=lLocalMapPoints.begin();
		while(lit != lLocalMapPoints.end())
		{
			ygomi::MapPoint* pMP = *lit;
			if(!pMP)
			{
				lit = lLocalMapPoints.erase(lit);
				
				fout << "mp is bad, continue.\n";
				
				continue;
			}
			
			fout << "mp ID : " << pMP->getId() << "\n";
			
			g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
			vPoint->setEstimate(algo::toVector3d(pMP->getPosition()));
			int id = pMP->getId() + maxKFid + 1;
			
			fout << "vertex id : " << id << "\n";
			
			vPoint->setId(id);
			vPoint->setMarginalized(true);
			optimizer.addVertex(vPoint);
			
			std::vector<ygomi::Track> observations = pMP->getObservation();
			std::sort(observations.begin(), observations.end(),
					  [&](const ygomi::Track& t1, const ygomi::Track& t2)->bool{return t1.m_frameId < t2.m_frameId;});
			
			int nObs = observations.size();
			
			fout << "track info: \n";
			for(auto& ob : observations) {
				fout << ob.m_frameId << " " << ob.m_keyPointId << "\n";
			}
			fout << "\n";
			
			//Set edges
			std::vector<ygomi::Track>::iterator mit = observations.begin();
			while(mit != observations.end())
			{
				long kfId = mit->m_frameId;
				ygomi::Frame* pKFi = m_pGlobalMap->getFrame(kfId);
				
				int count = 0;
				for(int i=0;i<vertexId.size();i++) {
					if(kfId == vertexId[i])
						count++;
				}
				if(count==0) {
					mit++;
					
					fout << "track is invalid, continue.\n";
					
					continue;
				}
				
				if( !pKFi ) {
					mit = observations.erase(mit);
					
					fout << "kf is empty continue.\n";
					
					continue;
				}
				
				if( pKFi->getType() != ygomi::BAD_KEY_FRAME ) {
					const cv::KeyPoint &kpUn = pKFi->getKeyPoint(mit->m_keyPointId).m_key;
					
					// Monocular observation
//					if(pKFi->mvuRight[mit->second]<0)
					{
						Eigen::Matrix<double,2,1> key2D;
						key2D << kpUn.pt.x, kpUn.pt.y;
						
						fout << "kp info : " << kpUn.pt.x << " " << kpUn.pt.y << " " << kpUn.octave << " " << kpUn.class_id << "\n";
						
						g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
						
						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->getId())));
						e->setMeasurement(key2D);
						const float &invSigma2 = invScaleFactorSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity()*invSigma2*(nObs));
						
						for(auto factor : invScaleFactorSigma2) {
							fout << factor << " ";
						}
						fout << "\n";
						fout << "weight : " << invSigma2 << " " << nObs << " " << invSigma2 * nObs << "\n";
						
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);
						
						e->fx = m_K.at<float>(0, 0); // pKFi->fx;
						e->fy = m_K.at<float>(1, 1);  //pKFi->fy;
						e->cx = m_K.at<float>(0, 2); // pKFi->cx;
						e->cy = m_K.at<float>(1, 2); // pKFi->cy;
						
						fout << e->fx << " " << e->fy << " " << e->cx << " " << e->cy << "\n";
						
						optimizer.addEdge(e);
						vpEdgesMono.push_back(e);
						vpEdgeKFMono.push_back(pKFi);
						vpMapPointEdgeMono.push_back(pMP);
					}
				}
				++mit;
			}
			++lit;
		}
		
//		if( &pbStopFlag ) {
//			if(pbStopFlag) {
//				if(g_switch_is_trace_multi_thread) std::cout << "LMPT: local ba stoped by pbStopFlag" << std::endl;
//				return;
//			}
//		}
		
		int nIterationNum = 20;

//		nIterationNum = m_pParamParser->parseInteger("");
		
		int iterNum = 0;
		
		optimizer.initializeOptimization();
		iterNum = optimizer.optimize(nIterationNum/*20*/);
		
		bool bDoMore= true;
		
//		if(pbStopFlag) {
//			if(*pbStopFlag)
//			{
//				bDoMore = false;
//				if(g_switch_is_trace_multi_thread) std::cout << "LMPT: ba do more flag become false by pbStopFlag" << std::endl;
//			}
//		}
		
		if(bDoMore)
		{
			
			// Check inlier observations
			for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
			{
				g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
				const ygomi::MapPoint* pMP = vpMapPointEdgeMono[i];
				
				if(!pMP)
				{
					// vpMapPointEdgeMono[i] = MPID::Invalid();
					continue;
				}
				
				if(pMP->isBad())
					continue;
				
				int nObs = pMP->getObservation().size();
				
//				std::cout << e->chi2() << std::endl;
				
				if(e->chi2()>5.991*(nObs) || !e->isDepthPositive())
				{
					e->setLevel(1);
				}
				
				e->setRobustKernel(0);
			}
			
			// Optimize again without the outliers
			
			optimizer.initializeOptimization(0);
			iterNum += optimizer.optimize(nIterationNum/*20*/);
			
		}
		
		std::vector<std::pair<long ,long> > vToErase; //kfId-mpId
		vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
		
		// Check inlier observations
		for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
		{
			g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
			const ygomi::MapPoint* pMP = vpMapPointEdgeMono[i];
			
			if(!pMP)
			{
				// vpMapPointEdgeMono[i] = MPID::Invalid();
				continue;
			}
			
			if(pMP->isBad())
				continue;
			
			int nObs = pMP->getObservation().size();
			if(e->chi2()>5.991*(nObs) || !e->isDepthPositive())
			{
				const ygomi::Frame* pKFi = vpEdgeKFMono[i];
				vToErase.push_back(std::make_pair(pKFi->getId(), pMP->getId()));
			}
		}
		
		// Get Map Mutex
		//unique_lock<mutex> lock(pMap->mMutexMapUpdate);
		for(size_t i=0; i<vToErase.size(); i++) {
			fout << vToErase[i].first << " " << vToErase[i].second;
		}
		fout << "\n";
		
		if(!vToErase.empty())
		{
			for(size_t i=0;i<vToErase.size();i++)
			{
				ygomi::Frame*    pKFi = m_pGlobalMap->getFrame(vToErase[i].first);
				ygomi::MapPoint* pMPi = m_pGlobalMap->getMapPoint(vToErase[i].second);
				if(!pKFi)
				{
					continue;
				}
				if(!pMPi)
				{
					continue;
				}
				
				auto& tracks = pMPi->getObservation();
				size_t kpId;
				bool bexist = false;
				for(auto& track : tracks) {
					if(track.m_frameId == pKFi->getId()) {
						kpId = track.m_keyPointId;
						bexist = true;
						break;
					}
				}
				CV_Assert(bexist);

				DelMpKpConnection(pMPi->getId(), pKFi->getId(), kpId);
			}
		}
		
		// Recover optimized data
		
		//Keyframes
		for(std::list<ygomi::Frame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
		{
			ygomi::Frame* pKF = m_pGlobalMap->getFrame((*lit)->getId());
			if(!pKF)
				continue;
			g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->getId()));
			g2o::SE3Quat SE3quat = vSE3->estimate();
			
			cv::Mat pose = algo::toCvMat(SE3quat);
			
			fout << pose << "\n";
			
			pose.convertTo(pose, CV_64FC1);
			
			
			pKF->setPose(pose.rowRange(0, 3));
		}
		
		std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
															   m_pParamParser->parseInteger("raulLevelNum"));
		//Points
		for(std::list<ygomi::MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
		{
			ygomi::MapPoint* pMP = m_pGlobalMap->getMapPoint((*lit)->getId());
			if(!pMP)
				continue;
			g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getId() + maxKFid + 1));
			const cv::Mat& pos = algo::toCvMat(vPoint->estimate());
			pMP->setPosition(cv::Point3f(pos.at<float>(0, 0), pos.at<float>(0, 1), pos.at<float>(0, 2)));
			
			std::vector<cv::Point3f> cameraCenters;
			auto& observations = pMP->getObservation();
			if(observations.empty())
				continue;
			int refKeyOctave = -1;
			for(auto& ob : observations) {
				ygomi::Frame* kf = m_pGlobalMap->getFrame(ob.m_frameId);
				cameraCenters.push_back(kf->getCameraCenter());
				
				if(ob.m_frameId == pMP->getReferenceKeyFrameID()) {
					refKeyOctave = kf->getKeyPoint(ob.m_keyPointId).m_key.octave;
				}
			}
			CV_Assert( -1 != refKeyOctave);
			pMP->updateNormalAndDepth(cameraCenters, refKeyOctave, scaleFactorSet);
			
			fout << "mpId : " << pMP->getId() << "\n";
			fout << "min dist : " << pMP->getMinDistance() << " max dist : " << pMP->getMaxDistance() << "\n";
			fout << "normal " << pMP->getNormal() << "\n";
		}
		
		fout.close();
//		if(g_switch_is_trace_multi_thread) std::cout << "LMPT: successful local BA for kf[" << GetIndice(pKF) << "] with iter [num=" << iterNum << "]" << std::endl;
	}
	
    void Platform::localBA(long frameId, int keyFrameCount)
    {
        const std::vector<ygomi::Frame*> ori_keyframes = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, keyFrameCount);
		
        std::unordered_map<long, ygomi::Frame*> keyframes;
        std::unordered_map<long, ygomi::MapPoint*> mappoints;

        for(auto& frame : ori_keyframes) {
            //1. save current key frame.
            keyframes.insert(std::make_pair(frame->getId(), frame));
			
            //2. get map points in key frame.
            const std::vector<ygomi::MapPoint*> temp_mps =  m_pGlobalMap->getMapPointInFrame(frame->getId());
            for(auto& mp : temp_mps) {
                if(mappoints.count(mp->getId())) //already exist.
                    continue;
                mappoints.insert(std::make_pair(mp->getId(), mp));
            }
        }

        //insert current key frame.
        keyframes.insert(std::make_pair(frameId, m_pGlobalMap->getFrame(frameId)));
        const std::vector<ygomi::MapPoint*> temp_mps =  m_pGlobalMap->getMapPointInFrame(frameId);
        for(auto& mp : temp_mps) {
            if(mappoints.count(mp->getId())) //already exist.
                continue;
            mappoints.insert(std::make_pair(mp->getId(), mp));
        }

        std::vector<float> invScaleFactorSigma2 = algo::calcScaleSquareSetInv(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                              m_pParamParser->parseInteger("raulLevelNum"));
		
        optimizeByLocalBA(keyframes,
                          mappoints,
                          m_K,
                          invScaleFactorSigma2,
                          m_pParamParser->parseInteger("nLocalBAOptimizeIterationNum"));
    }


    double Platform::computeCenterDis2_withPrev(long frameId)
    {
        std::vector<ygomi::Frame*> prev_kfs= m_pGlobalMap->getInterestKeyFrames(frameId, 1);
        ygomi::Frame* pre_kf = prev_kfs[0];
        if(!pre_kf || (pre_kf->getType() == ygomi::BAD_KEY_FRAME)){
            return 0;
        }

        ygomi::Frame* kf = m_pGlobalMap->getFrame(frameId);
        const cv::Point3f cam1 = kf->getCameraCenter();
        const cv::Point3f cam2 = pre_kf->getCameraCenter();

        double a = static_cast<float>(cam2.x) - static_cast<float>(cam1.x);
        double b = static_cast<float>(cam2.y) - static_cast<float>(cam1.y);
        double c = static_cast<float>(cam2.z) - static_cast<float>(cam1.z);

        double d2 = a * a + b * b + c * c;

        return d2;
    }

    void Platform::cullKeyFrame(long frameId)
    {
#ifdef Test_cullKeyFrame
        char ch_idx[256];
        memset(ch_idx, ' ', sizeof(ch_idx));
        sprintf(ch_idx, "%ld", frameId);

        const std::string& compare_root = "/Users/test/SLAM2.0Test/TestCaseCompare/cullkeyframe/gtest";
        const std::string& compare_path_matlab = compare_root + "/matlab_slam_output/case" + static_cast<std::string>(ch_idx) + "_matlab_slam_compare.txt";
        std::vector<int> keyIds, nmps, nRedundantObservationsVec;
        keyIds.reserve(20);
        nmps.reserve(20);
        nRedundantObservationsVec.reserve(20);
#endif

        //Get all key frames in covisibility graph
        std::vector<ygomi::Frame*> localKeyFrames = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, -1);

        for(auto& keyframe : localKeyFrames){
            if(keyframe->getId() == 0){
                continue;
            }
            const std::vector<long>& mapPointIndices = keyframe->getMapPointIndices();


            int nObservationTh = m_pParamParser->parseInteger("nKeyFrameCullingThObs");
            int nRedundantObservations = 0;
            int nMPs = 0;

            for(size_t i=0; i<mapPointIndices.size(); i++){
                const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mapPointIndices[i]);
                CV_Assert(mp);
                
                if(!mp->isBad()){
                    nMPs++;
                    const std::vector<ygomi::Track>& tracks = mp->getObservation();

                    if(tracks.size() <= nObservationTh){
                        continue;
                    }


                    // find the keyId
                    size_t keyId;
                    for(size_t j=0; j<tracks.size(); j++){
                        if(tracks[j].m_frameId == keyframe->getId())
                            keyId = tracks[j].m_keyPointId;
                    }
                    const int &scaleLevel = keyframe->getKeyPoint(keyId).m_key.octave;

                    int nObs = 0;
                    for(int k= tracks.size()-1; k >=0; k--){
                    	const ygomi::Frame* frame = m_pGlobalMap->getFrame(tracks[k].m_frameId);
                    	CV_Assert(frame);

                    	//
                    	if(frame->getId() == keyframe->getId()){

                        	continue;
                    	}
                    	//
                    	const int &scaleLeveli = frame->getKeyPoint(tracks[k].m_keyPointId).m_key.octave;

                    	if(scaleLeveli <= scaleLevel + 1){
                       		nObs++;
                       		if(nObs >= nObservationTh)
                          		break;
                        }

                   }

                   if(nObs>=nObservationTh){
                        nRedundantObservations++;
                   }
            }
        }

#ifdef Test_cullKeyFrame
            keyIds.push_back(keyframe->getId());
            nmps.push_back(nMPs);
            nRedundantObservationsVec.push_back(nRedundantObservations);
#endif

            if(nRedundantObservations>0.9*nMPs){
                // to do: add this two function
                double dis2 = computeCenterDis2_withPrev(keyframe->getId());
#ifndef Test_cullKeyFrame
                double mean_dis2 = kf_center_dis2_->get();
#endif

#ifdef Test_cullKeyFrame
                double mean_dis2 = 0.00638281; // only for frameId = 14
//                std::cout << "dis2: " << dis2 << std::endl;
//                std::cout << "mean_dis2: " << mean_dis2 << std::endl;
#endif
                if(dis2 > mean_dis2 * 5){
                    return;
                }

                keyframe->setType(ygomi::BAD_KEY_FRAME);
            }
        }

#ifdef Test_cullKeyFrame
        saveCullKeyFrameCout(keyIds, nmps, nRedundantObservationsVec, compare_path_matlab);
#endif
    }
}
