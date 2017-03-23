/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Tracker.cpp
 * @brief  Implementation of tracker module.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.04        Qiang.Zhai     Create
 *      2016.08.09        Zili.Wang      Add optimizePose module.
 *      2016.08.09        Zhongjun.Dai   Add predict pose
 *      2016.08.11        Zhongjun.Dai   Add decision key frame
 *******************************************************************************
 */

#include <stdio.h>
#include "Platform.hpp"
#include "FeatureExtractor.hpp"
#include "Frame.hpp"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include "TypeDef.hpp"
#include "ORBextractor.h"
#include "Optimizer.h"
#include "searchByProjection.hpp"
#include "paramParser.hpp"
#include "Utils.h"
#include <fstream>

//1st
void readSingleFrame(std::ifstream& fin,
                              long& frameId,
                              ygomi::FrameType& type,
                              cv::Mat& pose,
                              long& refID,
                              std::vector<ygomi::KeyPoint>& keys,
                              cv::Point3f& cameraCenter,
                              std::string& name);

void readSingleMapPoint(std::ifstream& fin,
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
void readSingleFrame(const std::string& path,
                            long& frameId,
                            ygomi::FrameType& type,
                            cv::Mat& pose,
                            long& refID,
                            std::vector<ygomi::KeyPoint>& keys,
                            cv::Point3f& cameraCenter,
                            std::string& name);

void readSingleMapPoint(const std::string& path,
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
                        std::vector<float>& scaleFactors, bool readExtra = false);

//3rd
ygomi::Frame* readSingleFrame(const std::string& path);
ygomi::MapPoint* readSingleMapPoint(const std::string& path);

//4th
ygomi::Frame* readSingleFrame(std::ifstream& fin);
ygomi::MapPoint* readSingleMapPoint(std::ifstream& fin);

//5th
std::vector<ygomi::Frame*> readAllFrames(const std::string& path);
std::vector<ygomi::MapPoint*> readAllMapPoints(const std::string& path);

static void readNextLine(std::ifstream& fin, std::istringstream& istr)
{
    std::string line;
    std::getline(fin, line);
    istr.clear();
    istr.str(line);
}

//1st cluster
void readSingleFrame(std::ifstream& fin,
                              long& frameId,
                              ygomi::FrameType& type,
                              cv::Mat& pose,
                              long& refID,
                              std::vector<ygomi::KeyPoint>& keys,
                              cv::Point3f& cameraCenter,
                              std::string& name)
{
    if(!fin.is_open()) {
        std::cout << "cannot open file stream " << std::endl;
        return;
    }

    //init
    keys.clear();
    
    //
    std::string str;
    std::istringstream istr;
    
    //frame id
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    frameId = atol(str.c_str());
    
    //refence key frame id.
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    refID = atol(str.c_str());
    
    //status
    readNextLine(fin, istr);
    istr >> str;
    bool isBad;
    istr >> str;    isBad = atoi(str.c_str());
    type = isBad ? ygomi::BAD_KEY_FRAME : ygomi::KEY_FRAME;
    
    //pose
    pose = cv::Mat(3, 4, CV_64FC1);
    readNextLine(fin, istr);
    for(int j=0; j<3; j++) {
        readNextLine(fin, istr);
        for(int i=0; i<4; i++) {
            istr >> str;
            pose.at<double>(j, i) = atof(str.c_str());
        }
    }
    readNextLine(fin, istr);
    readNextLine(fin, istr); //empty line
    
    //read camera center
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    cameraCenter.x = atof(str.c_str());
    istr >> str;    cameraCenter.y = atof(str.c_str());
    istr >> str;    cameraCenter.z = atof(str.c_str());
    
    //read keys.
    readNextLine(fin, istr);
    istr >> str;
    int keySize;
    istr >> str;    keySize = atoi(str.c_str());
    for(int i=0; i<keySize; i++) {
        readNextLine(fin, istr);
        cv::KeyPoint key_pt;
        istr >> str;    key_pt.pt.x     = atof(str.c_str());
        istr >> str;    key_pt.pt.y     = atof(str.c_str());
        istr >> str;    key_pt.octave   = atoi(str.c_str());
        istr >> str;    key_pt.class_id = atoi(str.c_str());
        
        std::vector<float> desc_val;
        readNextLine(fin, istr);//descriptor
        while(istr >> str)
            desc_val.push_back(atof(str.c_str()));
        
        cv::Mat desc(1, desc_val.size(), CV_32FC1);
        float* ptr = desc.ptr<float>(0);
        for(auto& val : desc_val)
            *ptr++ = val;
        
        readNextLine(fin, istr);//mp list num
        istr >> str;
        int mpListSize;
        std::vector<long> mpIDLists;
        istr >> str;    mpListSize = atoi(str.c_str());
        for(int i=0; i<mpListSize; i++) {
            readNextLine(fin, istr);
            istr >> str;
            long mpId = atol(str.c_str());
            
            mpIDLists.push_back(mpId);
        }
        
        ygomi::KeyPoint key;
        key.m_key        = key_pt;
        key.m_descriptor = desc;
        key.m_mapPointId = mpIDLists;
        key.m_descType   = ygomi::FLOAT_DESC;
        
        keys.push_back(key);
    }
}

void readSingleMapPoint(std::ifstream& fin,
                       bool& isBad,
                       long& mpId,
                       bool& mbTrackInView,
                       int& mnTrackScaleLevel,
                       float& mTrackViewCos,
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
                        std::vector<float>& scaleFactors, bool readExtra)
{
    if(!fin.is_open()) {
        std::cout << "cannot open source file\n";
        return;
    }
    
    std::string line, str;
    std::istringstream istr;
    
    //status
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    isBad = atoi(str.c_str());
    
    
    //id
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    mpId = atol(str.c_str());

    //mbTrackInView
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    mbTrackInView = atol(str.c_str());

    //mnTrackScaleLevel
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    mnTrackScaleLevel = atol(str.c_str());

    //mTrackViewCos
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    mTrackViewCos = atof(str.c_str());

    //pt
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    mp_pt.x = atof(str.c_str());
    istr >> str;    mp_pt.y = atof(str.c_str());
    istr >> str;    mp_pt.z = atof(str.c_str());
    
    //desc
    std::getline(fin, line);
    std::vector<float> desc_val;
    readNextLine(fin, istr);//descriptor.
    while(istr >> str)
        desc_val.push_back(atof(str.c_str()));
    
    descriptor = cv::Mat(1, desc_val.size(), CV_32FC1);
    float* ptr = descriptor.ptr<float>(0);
    for(auto& val : desc_val)
        *ptr++ = val;
    
    //track size
    tracks.clear();
    cameraCenters.clear();
    readNextLine(fin, istr);
    istr >> str;
    int trackSize;
    istr >> str;    trackSize = atoi(str.c_str());
    for(int i=0; i<trackSize; i++) {
        //read track
        readNextLine(fin, istr);
        long frameId;
        istr >> str;    frameId = atol(str.c_str());
        int keyId;
        istr >> str;    keyId = atoi(str.c_str());
        
        ygomi::Track track;
        track.m_frameId    = frameId;
        track.m_keyPointId = keyId;
        
        tracks.push_back(track);
        
        if(readExtra) {
            //read camera center.
            cv::Point3f center;
            readNextLine(fin, istr);
            istr >> str;    center.x = atof(str.c_str());
            istr >> str;    center.y = atof(str.c_str());
            istr >> str;    center.z = atof(str.c_str());
            cameraCenters.push_back(center);
        }
    }
    
    //first_kf_id
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    firstKFID = atol(str.c_str());
    
    //min_dist
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    minDistance = atof(str.c_str());
    
    //max_dist
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    maxDistance = atof(str.c_str());
    
    //norm vector
    readNextLine(fin, istr);
    istr >> str;
    
    normalVector = cv::Mat(3, 1, CV_32FC1);
    istr >> str;    normalVector.at<float>(0, 0) = atof(str.c_str());
    istr >> str;    normalVector.at<float>(1, 0) = atof(str.c_str());
    istr >> str;    normalVector.at<float>(2, 0) = atof(str.c_str());
    
    //visible
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    visible = atoi(str.c_str());
    
    //found
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    found = atoi(str.c_str());
    
    readNextLine(fin, istr);
    istr >> str;
    istr >> str;    referKFID = atol(str.c_str());
    
    //
    if(readExtra) {
        //
        readNextLine(fin, istr);
        istr >> str;
        istr >> str;    nlevel = atoi(str.c_str());
        
        readNextLine(fin, istr);
        
        scaleFactors.clear();
        readNextLine(fin, istr);
        while (istr >> str) {
            scaleFactors.push_back(atof(str.c_str()));
        }
        
    }
}


//2nd cluster
void readSingleFrame(const std::string& path,
                     long& frameId,
                     ygomi::FrameType& type,
                     cv::Mat& pose,
                     long& refID,
                     std::vector<ygomi::KeyPoint>& keys,
                     cv::Point3f& cameraCenter,
                     std::string& name)
{
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << "cannot open source file\n";
        return;
    }
    
    readSingleFrame(fin, frameId, type, pose, refID, keys, cameraCenter, name);
    
    fin.close();

}

void readSingleMapPoint(const std::string& path,
                        bool& isBad,
                        long& mpId,
                        bool& mbTrackInView,
                        int& mnTrackScaleLevel,
                        float& mTrackViewCos,
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
                        std::vector<float>& scaleFactors, bool readExtra)
{
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << "cannot open source file\n";
        return;
    }
    
    readSingleMapPoint(fin, isBad, mpId, mbTrackInView, mnTrackScaleLevel,
                       mTrackViewCos,mp_pt, descriptor, tracks, referKFID,
                       firstKFID, minDistance, maxDistance, normalVector,
                       visible, found, cameraCenters, nlevel, scaleFactors, readExtra);
    
    fin.close();
}

//3rd cluster
ygomi::Frame* readSingleFrame(std::ifstream& fin)
{
    ygomi::Frame* frame = nullptr;
    if(!fin.is_open()) {
        std::cout << " " << std::endl;
        return frame;
    }
    
    //read basic data from file.
    long frameId, refID;
    ygomi::FrameType type;
    cv::Mat pose;
    std::vector<ygomi::KeyPoint> keys;
    cv::Point3f cameraCenter;
    std::string name;
    readSingleFrame(fin,
                    frameId,
                    type,
                    pose,
                    refID,
                    keys,
                    cameraCenter,
                    name);
    
    frame = new ygomi::Frame(frameId);
    frame->setName(name);
    frame->setType(type);
    frame->setPose(pose);
    frame->addKeys(keys);
    frame->setRefFrameId(refID);
    
    return frame;
}

ygomi::MapPoint* readSingleMapPoint(std::ifstream& fin)
{
    ygomi::MapPoint* mp = nullptr;
    if(!fin.is_open()) {
        std::cout << " " << std::endl;
        return mp;
    }
    
    bool isBad;
    long mpID;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    cv::Point3f pt;
    cv::Mat descriptor;
    std::vector<ygomi::Track> tracks;
    long referKFID;
    long firstKFID;
    float minDistance;
    float maxDistance;
    cv::Mat normalVector;
    int visible;
    int found;
    std::vector<cv::Point3f> cameraCenters;
    int nlevel;
    std::vector<float> scalefactors;
    readSingleMapPoint(fin,
                       isBad,
                       mpID,
                       mbTrackInView,
                       mnTrackScaleLevel,
                       mTrackViewCos,
                       pt,
                       descriptor,
                       tracks,
                       referKFID,
                       firstKFID,
                       minDistance,
                       maxDistance,
                       normalVector,
                       visible,
                       found,
                       cameraCenters,
                       nlevel,
                       scalefactors,
                       true);
    
    
    //construct map point.
    mp = new ygomi::MapPoint(firstKFID);
    mp->setReferenceKeyFrameID(referKFID);
    mp->setBad(isBad);
    mp->setId(mpID);
    mp->setTrackInView(mbTrackInView);
    mp->setTrackScaleLevel(mnTrackScaleLevel);
    mp->setTrackViewCos(mTrackViewCos);
    mp->setPosition(pt);
    mp->setDescriptor(descriptor);
    mp->addTracks(tracks);
    
    mp->addVisible(-mp->getVisible());
    mp->addVisible(visible);
    
    mp->addFound(-mp->getRealFould());
    mp->addFound(found);
    
    return mp;
}

//4th cluster
ygomi::Frame* readSingleFrame(const std::string& path)
{
    ygomi::Frame* frame = nullptr;
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << " " << std::endl;
        return frame;
    }
    
    return readSingleFrame(fin);
}

ygomi::MapPoint* readSingleMapPoint(const std::string& path)
{
    ygomi::MapPoint* mp = nullptr;
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << " " << std::endl;
        return mp;
    }
    
    return readSingleMapPoint(fin);
}

//5th cluster
std::vector<ygomi::Frame*> readAllFrames(const std::string& path)
{
    std::vector<ygomi::Frame*> kfs;
    
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << " " << std::endl;
        return kfs;
    }
    
    std::istringstream istr;
    std::string str;
    readNextLine(fin, istr);
    istr >> str;
    int kf_num; istr >> kf_num;
    
    kfs.reserve(kf_num);
    for(int i=0; i<kf_num; i++) {
        kfs.push_back(readSingleFrame(fin));
    }
    
    return kfs;
}

std::vector<ygomi::MapPoint*> readAllMapPoints(const std::string& path)
{
    std::vector<ygomi::MapPoint*> mps;
    std::ifstream fin(path.c_str());
    if(!fin.is_open()) {
        std::cout << "  " << std::endl;
        return mps;
    }
    
    std::istringstream istr;
    std::string str;
    readNextLine(fin, istr);
    istr >> str;
    int mp_num; istr >> mp_num;
    mps.reserve(mp_num);
    for(int i=0; i<mp_num; i++) {
        mps.push_back(readSingleMapPoint(fin));
    }
    
    return mps;
}

namespace ygomi
{
    static const int MAX_SAVED_FRAME_NUM = 50;
    bool Platform::setFrame(long frameId, const char* path)
    {
        try {
            CV_Assert(path);
            if(m_frames.size() >= MAX_SAVED_FRAME_NUM) {
                //need to remove the oldest frame.
                long minFrameId = frameId;
                for(const auto& elem : m_frames) {
                    if(elem.first < minFrameId) {
                        minFrameId = elem.first;
                    }
                }
                CV_Assert(minFrameId < frameId);
                m_frames.erase(m_frames.find(minFrameId));
            }
            CV_Assert(!m_frames.count(frameId));
            
            std::string strPath(path);
            const std::string& frameName = strPath.substr(strPath.find_last_of('/')+1);
            
            //save current raw image
            cv::Mat img = cv::imread(path);
            if(img.empty()) {
                throw "Read frame failed\n";
            }
            m_frames[frameId] = img.clone();
            
            // add image width and height to paramParse
            const char* imgWidthParamName  = "ImageWidth";
            const char* imgHeightParamNema = "ImageHeight";
            if (!m_pParamParser->checkParam(imgWidthParamName)){
                m_pParamParser->addInteger(imgWidthParamName,  img.cols);
                m_pParamParser->addInteger(imgHeightParamNema, img.rows);
            }
            
            //construct a new Frame.
            ygomi::Frame* frame = new ygomi::Frame(frameId);
            frame->setType(ygomi::NORMAL_FRAME);
            frame->setName(frameName);
            
            m_pGlobalMap->addFrame(*frame);
        } catch(...) {
            std::cout << "Caught a general exception.\n";
            return false;
        }

        return true;
    }
    
    void Platform::analysisSearchByProjection(const ygomi::Frame* currframe,
                                              const std::vector<ygomi::MapPoint*> mapPoints,
                                              const std::vector<std::pair<size_t, size_t> >& matchingPairs)
    {
        static std::ofstream fout;
        static int count = 0;
        if(count++ >= 200) {
            if(fout.is_open())
                fout.close();
            return;
        }
        
        if(!fout.is_open()) {
            const std::string& descriptorType = m_pFeatureExtractor->getDescriptorType();
            const std::string path = descriptorType.substr(descriptorType.find_last_of(':')+1) + "_projection_track.xls";
            fout.open(path.c_str());
            fout << "total map points(3D)\t matched keys(2D)\t avgReprojectionError(pixel)\t medianReprojectionError(pixel)\t std\n";
        }
        
        std::vector<float> error;
        const auto& allKeys = currframe->getKeyPoints();
        for(const auto& key : allKeys) {
            const auto& mapIdIndices = key.m_mapPointId;
            for(auto id : mapIdIndices) {
                const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(id);
                CV_Assert(mp);
                
                if(mp->isBad())
                    continue;
                
                const cv::Point3f& pos = mp->getPosition();
                const cv::Mat& pose = currframe->getPose();
                cv::Mat posiM       = algo::pf3toMat(pos);
                cv::Mat ptHomo      = m_K64f*pose*posiM;
                float repX          = ptHomo.at<double>(0)/ptHomo.at<double>(2);
                float repY          = ptHomo.at<double>(1)/ptHomo.at<double>(2);
                cv::Point2f reprojPt(repX, repY);
                
                float dist = cv::norm(reprojPt - key.m_key.pt);
                error.push_back(dist);
            }
        }
        
        std::sort(error.begin(), error.end(), [](float a, float b)->bool{ return a < b;});
        cv::Scalar mean;
        cv::Scalar std;
        cv::meanStdDev(error, mean, std);
        
        fout << mapPoints.size()      << "\t";
        fout << matchingPairs.size()  << "\t";
        fout << mean.val[0]           << "\t";
        fout << error[error.size()/2] << "\t";
        fout << std.val[0]            << "\t";
        fout << "\n";
        
        
        
    }
	
	void Platform::refineObservations(long frameId)
	{
		
		ygomi::Frame* lastFrame = m_pGlobalMap->findLastFrame(frameId);
		if(!lastFrame || lastFrame->getType() != ygomi::KEY_FRAME)
			return;
		
//		showTrack(lastFrame->getId(), "before", false);
		
		std::vector<ygomi::Frame*> kfs(1, lastFrame);
		for(auto& kf : kfs) {
			CV_Assert(kf);
			const std::vector<long>& mpIndices = kf->getMapPointIndices();
			if(mpIndices.empty())
				continue;
			
			std::vector<std::pair<long, float> > trackDist;
			for(auto& mpId : mpIndices) {
				ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
				CV_Assert(mp);
				
				std::vector<ygomi::Track> tracks = mp->getObservation();
				//if(tracks.size() <= 1)
				//	continue;
				
				std::sort(tracks.begin(), tracks.end(),
						  [&](const ygomi::Track& t1, const ygomi::Track& t2)->bool{ return t1.m_frameId < t2.m_frameId; });
				int nTracksCount = static_cast<int>(tracks.size());
				int idx = -1;
				for(int n=0; n<nTracksCount; n++) {
					if(tracks[n].m_frameId == kf->getId()) {
						idx = n;
						break;
					}
				}
				
				CV_Assert(idx > 0);
				const auto& prevTrack = tracks[idx-1];
				const ygomi::Frame* prevFrame = m_pGlobalMap->getFrame(prevTrack.m_frameId);
				CV_Assert(prevFrame);
				const cv::KeyPoint& prevKey	  = prevFrame->getKeyPoint(prevTrack.m_keyPointId).m_key;
				
				const auto& currTrack = tracks[idx];
				const ygomi::Frame* currFrame = m_pGlobalMap->getFrame(currTrack.m_frameId);
				CV_Assert(currFrame);
				const cv::KeyPoint& currKey   = currFrame->getKeyPoint(currTrack.m_keyPointId).m_key;
				
				float dist = cv::norm(currKey.pt-prevKey.pt);
				trackDist.push_back(std::pair<long, float>(mpId, dist));
			}
			
			std::sort(trackDist.begin(), trackDist.end(),
					  [&](const std::pair<long, float>& p1, const std::pair<long, float>& p2)->bool{ return p1.second < p2.second; });
			
			std::vector<float> dist;
			for(size_t i=0; i<trackDist.size(); i++) {
				dist.push_back(trackDist[i].second);
			}
			
			cv::Scalar totalMean, totalSTD;
			cv::meanStdDev(dist, totalMean, totalSTD);
			
//			std::cout << "total mean : " << totalMean << " std : " << totalSTD << " median : " << trackDist[trackDist.size()/2].second << std::endl;
			
			dist.resize(dist.size()*0.25);
			cv::Scalar localMean, localSTD;
			cv::meanStdDev(dist, localMean, localSTD);
			
//			std::cout << "local mean : " << localMean << " std : " << localSTD << " median : " << trackDist[trackDist.size()/2].second << std::endl;
//			std::cout << "diff " << fabs(localMean.val[0]-totalMean.val[0]) << std::endl;
			
			
			if(localMean.val[0] < 2 && fabs(localMean.val[0]-totalMean.val[0]) > 3) {
				for(size_t i=0; i<trackDist.size()/2; i++) {
					ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(trackDist[i].first);
					
					ygomi::Track track;
					if(mp->getSingleObservation(kf->getId(), track)) {
						DelMpKpConnection(mp->getId(), kf->getId(), track.m_keyPointId);
					}
				}
			}
		}
		
		if(lastFrame->getType() == ygomi::KEY_FRAME)
			m_pGlobalMap->updateCovisibilityGraph(lastFrame->getId());
		
//		showTrack(lastFrame->getId(), "after", false);
	}
    
    bool Platform::track(long frameId)
    {
        //setImage must be called before Tracking.
        CV_Assert(m_pGlobalMap->getFrame(frameId));

        //extract features of current frame.
        extractFeatures(frameId);
		
		static TrackingState state(NOT_INITIALIZED);
		if(state == NOT_INITIALIZED) {
			bool bOK = tryInitPose(frameId);
			if(bOK) {
				m_referKFID = frameId;
				state = INITIALIZED;
			}
			
			std::cout << (bOK ? "init success\n" : "init failed\n");
		}
        else {
			bool bOK = false;
			
			//refineObservations(frameId);
			
			const bool bAutoPause = false;
//            if(frameId <= 3) {
//                bOK = trackReferenceKeyFrame(frameId);
//            }
//            else
			{
                //====== Track With Motion Model ======
                bOK = trackWithMotionModel(frameId);
				
//                if(!bOK)
//                    bOK = trackReferenceKeyFrame(frameId);
            }
			
            //====== Track Local Map ======
            if(bOK) {
               // bOK = trackLocalMap(frameId);
//                if (!optimizePose(frameId)) {
//					std::cout << "optimized pose failed after tracking local map\n";
//                    return false;
//                }
			}
			
            decisionKeyFrame(frameId);
			
			state = bOK ? TRACKING_SUCCESS : TRACKING_LOST;
			
//			if(state == TRACKING_LOST) {
//				state = NOT_INITIALIZED;
//				m_pGlobalMap->reset();
//			}
        }
		
		return state == TRACKING_SUCCESS;
    }
    
    void Platform::extractFeatures(long frameId)
    {
        CV_Assert(m_frames.count(frameId));
        const cv::Mat& img = m_frames.find(frameId)->second;
		
		//add mask
		const float scale = 1.0f/6;
		static const cv::Mat& mask = cv::Mat::zeros(img.rows * scale, img.cols, img.type());
		cv::Mat workingImg = img.clone();
        // TODO: comparison should be done whether mask is positive to system or not.
        //		mask.copyTo(workingImg.rowRange(workingImg.rows - mask.rows, workingImg.rows));

		//compute keys and descriptors.
		std::vector<ygomi::KeyPoint> keys;
        if(m_pFeatureExtractor)
            m_pFeatureExtractor->extract(workingImg, keys);
        
        //add frame to global map.
        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        frame->resetKeys();
        frame->addKeys(keys);
        frame->setRefFrameId(m_referKFID);
        
        m_pGlobalMap->addFrame(*frame);
    }
    
    void Platform::predictPose(long frameId)
    {
        Frame* lastFrame=m_pGlobalMap->findLastFrame(frameId);
        if(!lastFrame) {
            return;
        }
        Frame* lastlastFrame=m_pGlobalMap->findLastFrame(lastFrame->getId());
        if(!lastlastFrame) {
            return;
        }
//        int timeDeltaLast = lastFrame->getId() - lastlastFrame->getId();
//        int timeDeltaCur = frameId - lastFrame->getId();
//        float tiemDeltaRate = (float)timeDeltaCur/(float)timeDeltaLast; //to deal with case that the time duration is not constant.
        //todo consider this case in the future, now just simply believe the speed is constant;
        
        cv::Mat pose;
        cv::Mat lastHomo1 = cv::Mat::eye(4, 4, CV_64FC1);
        cv::Mat lastHomo2 = cv::Mat::eye(4, 4, CV_64FC1);
        cv::Mat invertHomo;
		if(lastFrame->getPose().empty()) {
			std::cout << "last pose empty\n";
		}

		if(lastlastFrame->getPose().empty()) {
			std::cout << "last last pose empty\n";
		}
        lastFrame->getPose().copyTo(lastHomo1.rowRange(0, 3));
        lastlastFrame->getPose().copyTo(lastHomo2.rowRange(0, 3));		
        cv::invert(lastHomo2, invertHomo);
        
        cv::Mat relativeHomo = lastHomo1 * invertHomo;
        cv::Mat currHomo = relativeHomo * lastHomo1;
        currHomo.rowRange(0, 3).copyTo(pose);

#ifdef Test_predictPose
        //===========================  test code (for comparison) =============================
        {
            std::cout << "\nCurrFile: " << __FILE__ << "\nCurrLine: " << __LINE__ << std::endl;
            cv::Mat currPose, prePose, invert;
            std::cout << "\nInput:\n";
            lastFrame->getPose().convertTo(currPose, CV_32FC1);
            lastlastFrame->getPose().convertTo(prePose, CV_32FC1);
            invertHomo.convertTo(invert, CV_32FC1);
            std::cout << "\nCurr Pose(transfor to CV_32FC1): \n" << currPose <<std::endl;
            std::cout << "\nPre Pose(transfor to CV_32FC1): \n" << prePose <<std::endl;
            std::cout << "\nInv(prePose)(transfor to CV_32FC1): \n" << invert << "\n"<<std::endl;
            cv::Mat relativeHomo32, pose32;
            relativeHomo.convertTo(relativeHomo32, CV_32FC1);
            pose.convertTo(pose32, CV_32FC1);
            std::cout << "\ncompute tranform matrix: " << "\n";
			std::cout << "\n'mVelocity'(transfor to CV_32FC1):\n" << relativeHomo32 << std::endl;
            std::cout << "\nPose(transfor to CV_32FC1): \n" << pose32 << "\n" << std::endl;
        }
        //===========================  test code (for comparison) =============================
#endif

        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        frame->setPose(pose);
    }
    
    void Platform::searchByProjection(long frameId, int keyFrameCount, float searchRadius, bool isImproveTh, std::vector<ygomi::MapPoint*>& mapPoints, std::vector<std::pair<long, size_t>>& matchingPairs)
    {
        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currframe);
        
        //get key frame indices.
//        const std::vector<ygomi::Frame*>& keyframes = m_pGlobalMap->getInterestKeyFrames(frameId, keyFrameCount);
        //get last frame
        ygomi::Frame* lastframe = m_pGlobalMap->findLastFrame(frameId);
		CV_Assert(lastframe);

        std::vector<ygomi::Frame*> frames;
        frames.push_back(lastframe);

        //get map point of interesting key frames.
//        std::vector<ygomi::MapPoint*> mapPoints;
//        mapPoints.clear();
//        for(const auto& frame : frames) {
//            const std::vector<ygomi::KeyPoint>& keys = frame->getKeyPoints();
//            for(const auto& key : keys) {
//                for(auto mapId : key.m_mapPointId) {
//                    ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mapId);
//                    CV_Assert(mp);
//                    if(std::find(mapPoints.begin(), mapPoints.end(), mp) == mapPoints.end())
//                        mapPoints.push_back(mp);
//                }
//            }
//        }
		
        //get 2d keys of current frame.
        const std::vector<ygomi::KeyPoint>& frameKeys = currframe->getKeyPoints();
        const std::vector<ygomi::KeyPoint>& lastframeKeys = lastframe->getKeyPoints();
        
        //get pose of current frame.
        const cv::Mat& pose = currframe->getPose();
        
        const cv::Rect imgRect = cv::Rect(0, 0, m_pParamParser->parseInteger("ImageWidth"), m_pParamParser->parseInteger("ImageHeight"));

//        long lastframeId = lastframe->getId();
        float scaleFactor = m_pParamParser->parseFloat("raulScaleFactor");
        int scaleLevelNum = m_pParamParser->parseInteger("raulLevelNum");
        TH_HIGH = 0.2;

#ifdef Test_searchByProjection
        //===========================  test code (for comparison) =============================
        {
            std::cout << "\nCurrFile: " << __FILE__ << "\nCurrLine: " << __LINE__ << std::endl;
            std::cout << "********** searchByProjection **********" << std::endl;
            std::cout << "mbCheckOrientation: " << mbCheckOrientation<< std::endl;
            std::cout << "frameKeys' size: " << frameKeys.size() << std::endl;
            std::cout << "CurrFrame Pose:\n " << pose << "\n" << std::endl;

        }
        //===========================  test code (for comparison) =============================
#endif



//        std::vector<std::pair<size_t, size_t> > matchingPairs;
//        ::searchByProjection(mapPoints,
//                             frameKeys,
//                             lastframeKeys,
//                             lastframeId,
//                             scaleFactor,
//                             scaleLevelNum,
//                             pose,
//                             m_K64f,
//                             imgRect,
//                             searchRadius,
//                             TH_HIGH,
//                             isImproveTh,
//                             matchingPairs,
//                             m_DistCoef);
		
		std::vector<std::pair<ygomi::MapPoint*, ygomi::KeyPoint> > lastMatchers;
		lastMatchers.reserve(lastframeKeys.size());
		for(const auto& key : lastframeKeys) {
			for(auto& mpId : key.m_mapPointId) {
				const auto& mp = m_pGlobalMap->getMapPoint(mpId);
				lastMatchers.push_back(std::pair<ygomi::MapPoint*, ygomi::KeyPoint>(mp, key));
			}
		}
		
		std::vector<float> scaleFactorSet = algo::calcScaleSet(scaleFactor, scaleLevelNum);
		float distThreshold = isImproveTh ? TH_HIGH * 2.0f : TH_HIGH;
		::searchByProjection(lastMatchers,
							 pose,
							 m_K64f,
							 frameKeys,
							 imgRect,
							 scaleFactorSet,
							 distThreshold,
							 searchRadius,
							 matchingPairs,
							 m_DistCoef);
		
//        showReprojection(frameId, false);
//        
//        analysisSearchByProjection(currframe, mapPoints, matchingPairs);
    }
    
    bool Platform::optimizePose(long frameId)
    {
        float fx = m_K.at<float>(0, 0);
        float fy = m_K.at<float>(1, 1);
        float cx = m_K.at<float>(0, 2);
        float cy = m_K.at<float>(1, 2);
        
        Frame* curFrame = m_pGlobalMap->getFrame(frameId);
		if(!curFrame)
			return false;
        cv::Mat pose = curFrame->getPose().clone();
        pose.convertTo(pose, CV_32FC1);
        
        std::vector<cv::Point3f> points3D;
        std::vector<cv::KeyPoint> currKeys;
        const std::vector<ygomi::KeyPoint>& kps = curFrame->getKeyPoints();
//        std::cout << "KpSize: " <<kps.size() << std::endl;
        std::vector<long> mpMapper;
        std::vector<int> kpMapper;
#if defined(Test_setPosePoseOptimi) || defined(Test_searchByProjection)
        //===========================  test code (for comparison) =============================
        std::vector<size_t> Indices;
        //===========================  test code (for comparison) =============================
#endif
        for(int i=0; i<kps.size(); i++){
            const KeyPoint& kp = kps[i];
            if(kp.m_mapPointId.empty())
                continue;

            int kpmpCount = kp.m_mapPointId.size();
//            std::cout << count++ <<"s kpmpCount: " << kpmpCount << std::endl;
            for (int j=0; j<kpmpCount; j++){
                MapPoint* mp= m_pGlobalMap->getMapPoint(kp.m_mapPointId[j]);
                if (mp->isBad()){
                    continue;
                }
                cv::Point3f posi = mp->getPosition();
                mpMapper.push_back(kp.m_mapPointId[j]);
                kpMapper.push_back(i);
                points3D.push_back(posi);
                currKeys.push_back(kp.m_key);
#if defined(Test_setPosePoseOptimi) || defined(Test_searchByProjection)
                //===========================  test code (for comparison) =============================
                Indices.push_back(i);
                //===========================  test code (for comparison) =============================
#endif
            }
        }

//        std::vector<float> invScaleSet = algo::calcScaleSetInv(m_pParamParser->parseFloat("raulScaleSigma2Factor"),
//															   m_pParamParser->parseInteger("raulLevelNum"));
		
		std::vector<float> invScaleSquare = algo::calcScaleSquareSetInv(m_pParamParser->parseFloat("raulScaleFactor"),
																	   m_pParamParser->parseInteger("raulLevelNum"));

		
		
        ///optimize pose
        const int keyNum = currKeys.size();
        if (keyNum <20){
            return false;
        }
        std::vector<bool> outliers(keyNum, false);
        std::vector<float> cameraParam = {fx, fy, cx, cy};
        cv::Mat updatedPose;
        ORB_SLAM2::Optimizer::PoseOptimization(
                                               cameraParam,     //camera intrinsic parameters
                                               pose,            //initial coarse pose
                                               points3D,        //map points position
                                               currKeys,        //keys in 2D after undistortion
                                               invScaleSquare,  //scale level
                                               updatedPose,     //optimized pose
                                               outliers);       //outliers flag
        numValidMapMatches = outliers.size();
        for (int i=0; i<outliers.size(); i++){
            if(outliers[i]){
                DelMpKpConnection(mpMapper[i], frameId,kpMapper[i]);
                numValidMapMatches--;
            }
        }
        if(updatedPose.empty()){
            return false;
        }
#if defined(Test_setPosePoseOptimi) || defined(Test_searchByProjection)
        //===========================  test code (for comparison) =============================
        {
            std::cout << "\n***Pose Optimization***: " << "\n";
            std::cout << "******************** INPUT **********************:" << "\n";
            std::cout << "camera param{fx, fy, cx, cy}: " << "\n" ;
            std::cout << cameraParam[0] << " " << cameraParam[1] << " " << cameraParam[2] << " " << cameraParam[3] << "\n";
            std::cout << "\npose:" << "\n";
            std::cout << pose << "\n";
            std::cout << "Pose's CV_MAT_TYPE: " << pose.type() << std::endl;
            std::cout << "\npoints3D: \n";
            for(size_t i=0; i<points3D.size(); i++){
                std::cout << "Curr refKp Index: " << Indices[i] << std::endl;
                std::cout << "points3D["<< i << "]: "<< points3D[i] << "\n";
            }
            std::cout << "\ncurrKeys: \n";
            for(size_t i=0; i<currKeys.size(); i++){
            	std::cout << "Curr Absolute Index: " << Indices[i] << std::endl;
                std::cout << "currKeys["<< i << "]'s position: "<< currKeys[i].pt << "\n";
                std::cout << "currKeys["<< i << "]'s octave: " << currKeys[i].octave << "\n";
            }
            std::cout << "\ninvScaleSet:" << "\n";
            for(auto scale : invScaleSet)
                std::cout << scale << "\n";

            std::cout << "\n******************** OUTPUT**********************:" << "\n";
            std::cout << "Optimized Pose: \n" << updatedPose << std::endl;
            std::cout << "Pose's CV_MAT_TYPE(has been convert to CV_64FC1): " << updatedPose.type() << std::endl;
            
            std::cout << "ValidMapMatches's num: " << numValidMapMatches << std::endl;
        }

        //===========================  test code (for comparison) =============================
#endif
        curFrame->setPose(updatedPose.rowRange(0, 3).clone());
        return true;
    }
    
    void Platform::searchLocalPoints(long frameId, const std::vector<ygomi::MapPoint*>& plocalMapPoints)
    {
        // get map points already matched
        std::vector<long> matchedMP;
        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currframe);
        const std::vector<ygomi::KeyPoint>& keys = currframe->getKeyPoints();
        for(const auto& key : keys) {
            for(auto id : key.m_mapPointId) {
                ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(id);
                if (!mp->isBad()) {
                    mp->addVisible(1);
                    mp->m_nLastFrameSeen = currframe->getId();
                    mp->m_mbTrackInView = false;
                    
                    if (std::find(matchedMP.begin(), matchedMP.end(), id) == matchedMP.end()) {
                        matchedMP.push_back(id);
                    }
                }
            }
        }

        //parse params
        cv::Rect imgRect = cv::Rect(0, 0, m_pParamParser->parseInteger("ImageWidth"), m_pParamParser->parseInteger("ImageHeight"));
        float ScaleFactor = m_pParamParser->parseFloat("raulScaleFactor");
        std::vector<float> scaleFactorSet = algo::calcScaleSet(ScaleFactor, m_pParamParser->parseInteger("raulLevelNum"));
        int maxLevel = m_pParamParser->parseInteger("raulLevelNum");

        //project points in local map and check its visibility.
//        std::cout << "plocalMapPoints size: " << plocalMapPoints.size() << "\n";
        std::vector<ygomi::MapPoint*> searchedMapPoints;
//==========================================================================================
//#define Test_SearchLocalPoints_Inner
#if  defined(Test_SearchLocalPoints_Inner)
        static int case_index   = 0;
        char ch_idx[256];
        memset(ch_idx, ' ', sizeof(ch_idx));
        sprintf(ch_idx, "%d", case_index);
        const std::string& inner_path = "/Users/test/Project/Slam_Matlab_Version/v2.0/cplusplus/unit_test/coretest/platform/Find3D2DPairsByProjection/Find3D2DInTrackLocalMap/testcase_plt/TmpToMatch" + static_cast<std::string>(ch_idx) + "_inner.txt";
        
        std::ofstream fout(inner_path.c_str());
        if(!fout.is_open()) {
            std::cout << "cannot open output stream\n";
            return;
        }
        fout << "mpId" << "\n";
#endif
//==========================================================================================
        int nToMatch=0;
        for(auto& mp : plocalMapPoints) {
            if (mp->m_nLastFrameSeen == currframe->getId() || std::find(matchedMP.begin(), matchedMP.end(), mp->getId()) != matchedMP.end()) {
                continue; //do not search map points already matched.
            }
            
            if(mp->isBad()) {
                continue;
            }
            
            //current map point does not pass valid check.
            if(!m_pGlobalMap->isInFrustum(frameId, mp->getId(), m_K, imgRect, 0.5, ScaleFactor, maxLevel)) {
                continue;
            }
            else{
                ygomi::MapPoint* mpG = m_pGlobalMap->getMapPoint(mp->getId());
                mpG->addVisible(1);
                searchedMapPoints.push_back(mpG);
                nToMatch++;
                // NOTE!!! now can not use the method below, because they are in two different memory. Tobe optimized later.
//                mp->addVisible(1);
//                searchedMapPoints.push_back(mp);
//                nToMatch++;
//==========================================================================================
#if  defined(Test_SearchLocalPoints_Inner)
                fout << mp->getId() << "\n";
#endif
//==========================================================================================
            }
        }
//==========================================================================================
#if  defined(Test_SearchLocalPoints_Inner)
        std::cout << "nToMatch= " << nToMatch << "\n";
        fout << "\n" << "nToMatch= " << nToMatch << "\n";
        fout.close();
        case_index++;
#endif
//==========================================================================================

        if(searchedMapPoints.size() > 0){
        	//get current camera pose.
        	const cv::Mat& pose = m_pGlobalMap->getFrame(frameId)->getPose();
        	const float mfNNRatio = m_pParamParser->parseFloat("fTrackLocalMapNNRatio");
            
            // get tracks size of all map points.
            const std::vector<ygomi::MapPoint*> mps = m_pGlobalMap->getAllMapPoints();
            std::map<int, int> tracksSize;
            for (int i = 0; i < mps.size(); i++) {
                if (mps[i]->isBad()) {
                    continue;
                }
                tracksSize.insert(std::make_pair(mps[i]->getId(), mps[i]->getObservation().size()));
            }

            //search by projection.
            std::vector<std::pair<long, size_t> > matchingPairs;
            ::searchByProjection(searchedMapPoints, keys, pose, m_K64f, imgRect, scaleFactorSet, mfNNRatio, tracksSize, matchingPairs);
     
    	    //update connections between map points and key points.
        	for(const auto& pair : matchingPairs) {
            	long mapId   = searchedMapPoints[pair.first]->getId();
            	size_t keyId = pair.second;
            	currframe->addMatchingPair(keyId, mapId);
            	//NOTE!!! Tracks are added only when frame is a key frame.
        	}
        }
    }

    float Platform::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        cv::Mat disMat = a - b;
//        std::cout << "disMat: " << a << "\n"
//      			  << b << "\n"
//                  << disMat  << std::endl;

        float dist = cv::norm(disMat);

        float d2 = dist * dist;
        
        return d2;
    }

    void Platform::ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }

    void Platform::searchByBoW(ygomi::Frame* refKeyFrame,
                               ygomi::Frame* currFrame,
                               const std::vector<ygomi::MapPoint*>& mapPoints,
                               std::vector<std::pair<size_t, size_t> > &matchingPairs)
    {
        // parse params
        float fNNRatio = m_pParamParser->parseFloat("fTrackReferenceKeyFrameNNRatio"); //default 0.7
        float fSGDMatchThHigh = m_pParamParser->parseFloat("fSGDMatchThHigh"); //default 0.05
        float fSGDMatchThLow = m_pParamParser->parseFloat("fSGDMatchThLow"); //default 0.5
        float TH_HIGH = fSGDMatchThHigh * TH_MAX;
        float TH_LOW = fSGDMatchThLow * TH_HIGH;
		#ifdef Test_setPosePoseOptimi
        //===========================  test code (for comparison) =============================
        {
            std::cout << "***Params***: " << "\n";
            std::cout << "mbCheckOrientation: " << mbCheckOrientation << "\n";
            std::cout << "HISTO_LENGTH: " << HISTO_LENGTH << "\n";
            std::cout << "TH_HIGH: " << TH_HIGH << "\n";
            std::cout << "TH_LOW: " << TH_LOW << "\n";
            std::cout << "mfNNratio: " << fNNRatio << "\n\n";
            std::cout << "***Output in searchByBoW***:" << "\n";
        }
        //===========================  test code (for comparison) =============================
		#endif


        std::vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;


        matchingPairs.clear();
        const size_t nRefKpSize = refKeyFrame->getKeyPoints().size();
        const size_t nCurrKpSize = currFrame->getKeyPoints().size();


        //===========================  test code (for comparison) =============================
        int totalvalidMp = 0;
        //===========================  test code (for comparison) =============================
        for(size_t i=0; i<nRefKpSize; i++) {
            const cv::Mat &dKF = refKeyFrame->getKeyPoint(i).m_descriptor;
            const ygomi::KeyPoint &kp = refKeyFrame->getKeyPoint(i);
            if(kp.m_mapPointId.empty())
            {
                continue;
            }
            //===========================  test code (for comparison) =============================
            totalvalidMp ++;
            //===========================  test code (for comparison) =============================
            
            float bestDist1 = TH_MAX;
            float bestDist2 = TH_MAX;
            int bestIdxF = -1;


            for (size_t j = 0; j < nCurrKpSize; j++)
            {
                const cv::Mat &dF = currFrame->getKeyPoint(j).m_descriptor;

                const float dist = DescriptorDistance(dKF, dF);

                if (dist < bestDist1)
                {
                    bestDist2 = bestDist1;
                    bestDist1 = dist;
                    bestIdxF = j;
                }
                else if (dist < bestDist2)
                {
                    bestDist2 = dist;
                }
            }
            #ifdef Test_setPosePoseOptimi
            //===========================  test code (for comparison) =============================
            std::cout << "KP[" << i << "]'s bestDist1: " << bestDist1 << std::endl;
            //===========================  test code (for comparison) =============================
			#endif
            if (bestDist1 <= TH_LOW)
            {
                if (static_cast<float>(bestDist1) < fNNRatio * static_cast<float>(bestDist2))
                {
                    const ygomi::KeyPoint &kp = refKeyFrame->getKeyPoint(i);

                    if (mbCheckOrientation){
                        float rot = kp.m_key.angle - currFrame->getKeyPoint(bestIdxF).m_key.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdxF);
                    }
                    // when match, add the Map points to the current matchingPairs
                    for(long k =0; k<mapPoints.size(); k++)
                    {
//                        for(auto mpId : kp.m_mapPointId)
//                        {
//                          std::cout << "KP's mpId: " << mpId << "\n"
//                                    << "map["<< k <<"]'s mpId: " << mapPoints[k]->getId() << "\n";
//                        }
                        const std::vector<long>& mapPointIdSet = kp.m_mapPointId;
                        if(std::find(mapPointIdSet.begin(), mapPointIdSet.end(), mapPoints[k]->getId()) != mapPointIdSet.end())
                        {
							matchingPairs.push_back(std::make_pair(mapPoints[k]->getId(), bestIdxF));
                        }

                    }
                }
            }
        }

        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    // remove the pair
                    for(size_t pairId=0; pairId<matchingPairs.size(); pairId++)
                    {
                        std::pair<size_t, size_t> currPair = matchingPairs[pairId];
                        int keyId = currPair.second;
                        std::cout << "currPair's second item(currframe's KpId): " << keyId << "\n"
                                  << "rotHist[i],[j]: " << rotHist[i][j] << "\n";
                        if(rotHist[i][j] == keyId)
                        {
                            matchingPairs.erase(matchingPairs.begin()+pairId);
                            // once a pair has been removed, the order will change
                            pairId--;
                        }

                    }
                }
            }
        }
        std::cout << "mapPoints Num:" << mapPoints.size() << std::endl;
        //update connections between map points and key points.
        for(const auto& pair : matchingPairs) {
            long mapId = pair.first;
            size_t keyId = pair.second;
            currFrame->addMatchingPair(keyId, mapId);
            //NOTE!!! Track of map point(3D) is added only when current frame is
            //a key frame, so we just add matching pair of key point(2D) here.
        }
        #ifdef Test_setPosePoseOptimi
        //===========================  test code (for comparison) =============================
        {
            std::cout << "\n***Brute-force matches's Number***: " << matchingPairs.size() << "\n";
            std::cout << "\n***TotalValidMp***: "<< totalvalidMp << "\n" << std::endl;
        }
        //===========================  test code (for comparison) =============================
		#endif
    }
    
    bool Platform::trackReferenceKeyFrame(long frameId)
    {

        // it will be used when choose ORB
        std::string descriptorType = m_pParamParser->parseString("DescriptorType");
        if(!strcmp("ORB", descriptorType.c_str())){
       		 mbCheckOrientation = true;
        }else if (!strcmp("SGD", descriptorType.c_str())){
       		 mbCheckOrientation = false;
        }


        std::vector<std::pair<size_t, size_t> > matchingPairs;
        // For SGD, not use BOW currently
        ygomi::Frame* pCurrFrame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(pCurrFrame);
        // May need to modify: How to find Reference Key frame, here previous frame is a key frame
        ygomi::Frame* pRefKeyFrame = m_pGlobalMap->getFrame(m_referKFID);
        CV_Assert(pRefKeyFrame->getType() == ygomi::KEY_FRAME);

#ifndef Test_setPosePoseOptimi
        //get map points of Reference key frames.
        std::vector<ygomi::MapPoint*> mapPoints;
        const std::vector<ygomi::KeyPoint>& keys = pRefKeyFrame->getKeyPoints();
        for(const auto& key : keys) {
            for(auto mapId : key.m_mapPointId) {
                ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mapId);
                CV_Assert(mp);
                if(std::find(mapPoints.begin(), mapPoints.end(), mp) == mapPoints.end())
                    mapPoints.push_back(mp);
            }
        }

        searchByBoW(pRefKeyFrame, pCurrFrame, mapPoints, matchingPairs);
        int nmatches = matchingPairs.size();
        if(nmatches < 15){
            return false;
        }
#endif
        
        // pose here: 3 x 4 ;  vehicle pose: 4 x 4 ;
        pCurrFrame->setPose(pRefKeyFrame->getPose());
        // UpdatePoseMatrices() was not added here, since we don't use rotation, translation etc. here.

		#ifdef Test_setPosePoseOptimi
        //===========================  test code (for comparison) =============================
        {
            std::cout << "\n***Output of SetPose***: " << "\n";
            std::cout << "Set Pose: " << "\n";
            std::cout << pRefKeyFrame->getPose() << std::endl;
            std::cout << "Pose's CV_MAT_TYPE: " << pRefKeyFrame->getPose().type() << std::endl;
        }

        //===========================  test code (for comparison) =============================
		#endif

        // Discard outliers procedure has been involved
        optimizePose(frameId);

        // judge the the valid map points
        return numValidMapMatches > 10;
    }

    bool Platform::trackWithMotionModel(long frameId)
    {
        predictPose(frameId);
        const int keyFrameCount = 1;
        //track motion model.
        float searchRadius = m_pParamParser->parseFloat("thWnd_");
        //==========================================================================================
#ifdef Test_searchByProjection
        {
            static int case_index   = 0;
            char ch_idx[256];
            memset(ch_idx, 0, sizeof(ch_idx));
            sprintf(ch_idx, "%d", case_index);
            const std::string& input_kf_path1 = "/Users/test/SLAM2.0Test/TestCaseInput_txt/searchByProjection_LastFrame" + static_cast<std::	string>(ch_idx) + "_input.txt";
            const std::string& input_kf_path2 = "/Users/test/SLAM2.0Test/TestCaseInput_txt/searchByProjection_currFrame" + static_cast<std::	string>(ch_idx) + "_input.txt";
            const std::string& input_mp_path3 = "/Users/test/SLAM2.0Test/TestCaseInput_txt/searchByProjection_allMappoints" + static_cast<std::	string>(ch_idx) + "_input.txt";
            // add frame
            ygomi::Frame* lastframe = readSingleFrame(input_kf_path1);
            ygomi::Frame* currframe = readSingleFrame(input_kf_path2);
            m_pGlobalMap->reset();
            m_pGlobalMap->addFrame(*lastframe);
            m_pGlobalMap->addFrame(*currframe);

            std::vector<ygomi::MapPoint*> allMapPoints = ::readAllMapPoints(input_mp_path3);
            for(const auto& mp : allMapPoints)
                m_pGlobalMap->addMapPoint(*mp);

        }
#endif
        //==========================================================================================

        std::vector<std::pair<long, size_t> > matchingPairs;
        std::vector<ygomi::MapPoint*> mapPoints;
        searchByProjection(frameId, keyFrameCount, searchRadius, false, mapPoints, matchingPairs);
#ifndef Test_searchByProjection
        if(matchingPairs.size() < 20)
#endif
        {
        	searchByProjection(frameId, keyFrameCount, 2*searchRadius, true, mapPoints, matchingPairs);
        }

        //update connections between map points and key points.
        ygomi::Frame* currframe = m_pGlobalMap->getFrame(frameId);
        CV_Assert(currframe);
        for(const auto& pair : matchingPairs) {
			currframe->addMatchingPair(pair.second, pair.first);
            //NOTE!!! Track of map point(3D) is added only when current frame is
            //a key frame, so we just add matching pair of key point(2D) here.
        }

//        std::cout<<"track from last frame nmatches = "<< matchingPairs.size() << std::endl;

        if(matchingPairs.size() < 10)
            return false;

        optimizePose(frameId);
//        std::cout<<"track from last frame nGood nmatches = "<< numValidMapMatches << std::endl;

#ifdef Test_searchByProjection
        //===========================  test code (for comparison) =============================
        std::cout << "\nCurrFile: " << __FILE__ << "\nCurrLine: " << __LINE__ << std::endl;
        std::cout << "nmatchesMap Num: " << numValidMapMatches << std::endl;
        //===========================  test code (for comparison) =============================
#endif
    	return numValidMapMatches >= 10;

    }

    bool Platform::trackLocalMap(long frameId)
    {
        //get local map points.
        std::vector<ygomi::Frame*> localKeyFrames;
        std::vector<ygomi::MapPoint*> plocalMapPoints;
        
        m_pGlobalMap->getLocalMapPoints(frameId, plocalMapPoints, localKeyFrames);
        
        //search local map points by projection.
        searchLocalPoints(frameId, plocalMapPoints);
        
        return true;
    }
    
    void Platform::decisionKeyFrame(long frameId)
    {
        bool bNeedKF = false;
        ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
        CV_Assert(frame);
        
        const int nKFs = m_pGlobalMap->getAllKeyFrames().size();
        
        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if(nKFs <= 2)
        {
            nMinObs = 2;
        }
        Frame* refFrame = m_pGlobalMap->getFrame(m_referKFID);
        CV_Assert(refFrame);
        int nRefMatches = trackedMapPoints(nMinObs, refFrame);
		
        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = true;

        // Thresholds
        float thRefRatio = 0.9;
        int maxFrames = 10;
        int minFrames = 5;
        int thMatchedMaxNum = 500;
        int thMatchedMinNum = 150;
        int nMatchesInliers = trackedMapPoints(0, frame); // TODO: add nMatchesInliers in trackLocalMap, need outliers judgemfent of key points

        // Condition c0: tracking very good or very bad, do not need to insert keyframes
        const bool c0 = ((nMatchesInliers <= 15 ) || (nMatchesInliers >= nRefMatches * thRefRatio));
        
        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const std::vector<const ygomi::Frame*> kfSet = m_pGlobalMap->getAllKeyFrames();
        const Frame* kfLast = kfSet.back();
        const bool c1a = frame->getId() >= kfLast->getId() +  maxFrames;
        
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (frame->getId() >= kfLast->getId() + minFrames && bLocalMappingIdle);
        
        //Condition 1c: tracking is weak, from tracking dimensional
        const bool c2a = ((nMatchesInliers < nRefMatches * thRefRatio) && (nMatchesInliers < 150)); // wether inliners is somewhat few.
        const bool c2b = nMatchesInliers < 80;  // wether inliners is too few.
        
        const bool c3 = nMatchesInliers < thMatchedMaxNum;
        const bool c4 = nMatchesInliers < thMatchedMinNum;
        
        
        if(c2a || c4)
        {
            // if inliners is somewhat few, do something
            bNeedKF = true;
        }
        else if((c1a||c1b) && (!c0 && c3))
        {
            if (bLocalMappingIdle)
            {
                bNeedKF = true;
            }
            else
            {
                bNeedKF = false;
            }
        }
        else
        {
            bNeedKF = false;
        }
		
//		bNeedKF = true;
        std::cout << "bNeedKF= " << bNeedKF << "\n";
		
        if(bNeedKF)
        {
			
			std::cout << frameId <<  " is a key frame!!!!\n";

            m_referKFID = frameId;
            frame->setRefFrameId(m_referKFID);
            
            //add current frame as a key frame.
            frame->setType(ygomi::KEY_FRAME);
            m_pGlobalMap->addKeyFrame(frameId);
            
            //update map point.
            const std::vector<ygomi::KeyPoint>& keys = frame->getKeyPoints();
            for(size_t keyId=0; keyId < keys.size(); keyId++)
            {
                const ygomi::KeyPoint& key = keys[keyId];
				if(key.m_mapPointId.empty())
					continue;
				
				
				//for test
				for(auto id : key.m_mapPointId)
                {
					ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(id);
					const auto& tracks = mp->getObservation();
					
					for(const auto& track : tracks)
                    {
						CV_Assert(track.m_frameId != frameId);
					}
				}
				
				//make the unique connections.
				//multi-map point is projected to the same 2d key point is not allowed.
				int bestMPId		= -1;
				int maxTracksCount  = -1;
				if(key.m_mapPointId.size() == 1) {
					bestMPId = key.m_mapPointId[0];
				}
				else
                {
					for(auto id : key.m_mapPointId)
                    {
						const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(id);
						int nTrackSize = static_cast<int>(mp->getObservation().size());
						if(nTrackSize > maxTracksCount)
                        {
							bestMPId		= id;
							maxTracksCount	= nTrackSize;
						}
					}
				}

				CV_Assert(bestMPId >= 0);
				
				ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(bestMPId);
				
				frame->resetMatchingPair(keyId);
				frame->addMatchingPair(keyId, bestMPId);
				
				mp->addTrack(frameId, keyId);
				
                std::vector<cv::Point3f> cameraCenters;
                size_t refKeyOctave;
				std::vector<cv::Mat> descs;
				const std::vector<ygomi::Track>& tracks = mp->getObservation();
				for(const auto& track : tracks)
                {
					const ygomi::Frame* frame = m_pGlobalMap->getFrame(track.m_frameId);
					descs.push_back(frame->getKeyPoints()[track.m_keyPointId].m_descriptor);
                    
                    cameraCenters.push_back(frame->getCameraCenter());
                    if(track.m_frameId == mp->getReferenceKeyFrameID()) {
                        refKeyOctave = frame->getKeyPoint(track.m_keyPointId).m_key.octave;
                    }
				}
				mp->mergeDescriptor(descs);
                
                std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"),
                                                                       m_pParamParser->parseInteger("raulLevelNum"));
                mp->updateNormalAndDepth(cameraCenters, refKeyOctave, scaleFactorSet);
            }
			
            //todo: update covisibility graph.
			m_pGlobalMap->updateCovisibilityGraph(frameId);
        }
    }
	
    int Platform::trackedMapPoints(const int &minObs, Frame* refFrame)
    {
        int nPoints = 0;
        const bool bCheckObs = minObs >= 0;
        const std::vector<long> mpIdx = refFrame->getMapPointIndices();
        for(int i = 0; i < mpIdx.size(); i++)
        {
            const ygomi::MapPoint* mp = m_pGlobalMap->getMapPoint(mpIdx[i]);
            if(mp)
            {
                if(!mp->isBad())
                {
                    if(bCheckObs)
                    {
                        if(mp->getObservation().size() >= minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }
        
        return nPoints;
    }
}
