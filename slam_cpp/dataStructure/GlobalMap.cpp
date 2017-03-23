/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   GlobalMap.cpp
 * @brief  Implementation of class GlobalMap
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#include "GlobalMap.hpp"
#include "Frame.hpp"
#include "LocalMap.hpp"

//#define RUN_TEST

#include <fstream>
#include <sstream>

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
                               std::vector<float>& scaleFactors, bool readExtra = false);

//3rd
extern ygomi::Frame* readSingleFrame(const std::string& path);
extern ygomi::MapPoint* readSingleMapPoint(const std::string& path);

//4th
extern ygomi::Frame* readSingleFrame(std::ifstream& fin);
extern ygomi::MapPoint* readSingleMapPoint(std::ifstream& fin);

//5th
extern std::vector<ygomi::Frame*> readAllFrames(const std::string& path);
extern std::vector<ygomi::MapPoint*> readAllMapPoints(const std::string& path);

void readNextLine(std::ifstream& fin, std::istringstream& istr)
{
	std::string line;
	std::getline(fin, line);
	istr.clear();
	istr.str(line);
}

namespace ygomi {
    GlobalMap::GlobalMap()
    {
        clear();
    }
    
    GlobalMap::~GlobalMap()
    {
        clear();
    }
    
    void GlobalMap::clear()
    {
        MapPoint::m_nextId = 0;
        m_frames.clear();
        m_mapPoints.clear();
        m_keyFramesIndices.clear();
        m_covisibilityGraph.clear();
    }
    
    // Read/Write handle
    void GlobalMap::read(const char* path)
    {
        // todo:
    }
    
    void GlobalMap::write(const char* path) const
    {
        // todo:
    }
    
    void GlobalMap::reset()
    {
        clear();
    }

    // write global map.
    void GlobalMap::addFrame(const ygomi::Frame& frame)
    {
        if(m_frames.count(frame.getId())) {
            updateFrame(frame);
        }
        else {
            m_frames.insert(std::make_pair(frame.getId(), frame));
        }
    }
    
    Frame* GlobalMap::addFrame(long frameId)
    {
        Frame fr(frameId);
        addFrame(fr);
        return getFrame(frameId);
    }
    
    void GlobalMap::updateFrame(const ygomi::Frame &frame)
    {
        CV_Assert(m_frames.count(frame.getId()));
        m_frames.find(frame.getId())->second = frame;
    }
    
    void GlobalMap::addMapPoint(const ygomi::MapPoint& mapPoint)
    {
        CV_Assert(!m_mapPoints.count(mapPoint.getId()));
        m_mapPoints.insert(std::make_pair(mapPoint.getId(), mapPoint));
    }
    
    void GlobalMap::removeMapPoint(long mapPointId)
    {
        CV_Assert(m_mapPoints.count(mapPointId));
        m_mapPoints.erase(m_mapPoints.find(mapPointId));
    }

    void GlobalMap::addKeyFrame(long frameId)
    {
        const ygomi::Frame* frame = getFrame(frameId);
        CV_Assert(frame && frameId == frame->getId());
        const auto& iter = std::find(m_keyFramesIndices.begin(), m_keyFramesIndices.end(), frameId);
        if(frame->getType() == ygomi::KEY_FRAME && iter == m_keyFramesIndices.end())
            m_keyFramesIndices.push_back(frameId);
    }
    
    void GlobalMap::addIndicesToRecentAddedMapPointsList(long mpId)
    {
        CV_Assert(m_mapPoints.count(mpId) &&
                  std::find(m_lRecentAddedMapPointsIndices.begin(),
                            m_lRecentAddedMapPointsIndices.end(),
                            mpId) == m_lRecentAddedMapPointsIndices.end());
        m_lRecentAddedMapPointsIndices.push_back(mpId);
        
    }
    
    void GlobalMap::updateSingleCovisibilityGraph(long frameId)
    {
        const ygomi::Frame* keyframe = getFrame(frameId);
        CV_Assert(keyframe && keyframe->getType() == KEY_FRAME);

        std::unordered_map<long, size_t> connections;
        for(const auto& key : keyframe->getKeyPoints()) {
            for(auto mapId : key.m_mapPointId) {
                if (!m_mapPoints.count(mapId)){
                    std::cout<<"assert(m_mapPoints.count(mapId))!!!!"<<"  mpId:" << mapId <<std::endl;
                    continue;
                }
                const auto& mp = m_mapPoints.find(mapId)->second;
                if(mp.isBad()) {
                    continue;
                }
                
                const std::vector<ygomi::Track>& obserations = mp.getObservation();
                for(const auto& track : obserations) {
                    if(track.m_frameId == frameId)
                        continue; //not including itself.
                    if(m_frames.find(track.m_frameId)->second.getType() != ygomi::KEY_FRAME)
                        continue;
                    if(connections.count(track.m_frameId)) {
                        connections[track.m_frameId]++;
                    }
                    else {
                        connections[track.m_frameId] = 1; //initialize.
                    }
                }
            }
        }
        
        //remove the connections below the threshold.
        std::vector<std::pair<long, size_t> > neighs(connections.begin(), connections.end());
        connections.clear();
        
        const int th = 15;
        int nmax = 0;
        long maxKFId;
        for(auto& neigh : neighs) {
            if(neigh.second > nmax) {
                nmax    = neigh.second;
                maxKFId = neigh.first;
            }
            if(neigh.second >= th) {
                connections.insert(neigh);
            }
        }
        
        if(connections.empty() && nmax > 0) {
            connections[maxKFId] = nmax;
        }
        
        if(!connections.empty())
            m_covisibilityGraph[frameId] = connections;
    }
    
    void GlobalMap::updateCovisibilityGraph(long frameId)
    {
        if(-1 == frameId) {
            updateAllKeyFrames();
            for(auto keyframeId : m_keyFramesIndices) {
                updateSingleCovisibilityGraph(keyframeId);
            }
        }
        else {
            //1. update current key frame
            updateSingleCovisibilityGraph(frameId);
            
            //2. update connections
            if(m_covisibilityGraph.count(frameId)) {
                const std::unordered_map<long, size_t> connections = m_covisibilityGraph.find(frameId)->second;
                for(const auto& c : connections) {
                    updateSingleCovisibilityGraph(c.first);
                }
            }
        }
    }

    void GlobalMap::updateAllKeyFrames()
    {
        m_keyFramesIndices.clear();
        m_keyFramesIndices.reserve(m_frames.size());
        for(auto it : m_frames) {
            CV_Assert(it.first == it.second.getId());
            if(it.second.getType() == ygomi::KEY_FRAME) {
                m_keyFramesIndices.push_back(it.first);
            }
        }
        std::sort(m_keyFramesIndices.begin(), m_keyFramesIndices.end(), [](long a, long b)->bool{return a < b;});
    }

    std::vector<ygomi::Frame*> GlobalMap::getBestCovisibilityKeyFrames(long frameId, int keyFrameCount)
    {
        std::vector<ygomi::Frame*> keyframes;
        CV_Assert(m_frames.count(frameId));
        CV_Assert(m_frames.find(frameId)->second.getType() == KEY_FRAME);
//        CV_Assert(m_covisibilityGraph.count(frameId));
		
		//todo: key frame may not be exist in covisibility graph, 2016.9.26
		//double-check other places.
		if(!m_covisibilityGraph.count(frameId)) {
			return keyframes;
		}
	
        const auto& graph = m_covisibilityGraph.find(frameId)->second;
        std::vector<std::pair<long, size_t> > connections(graph.begin(), graph.end());
        
        std::sort(connections.begin(), connections.end(),
                  [](const std::pair<long, size_t>& a, const std::pair<long, size_t>& b)->bool{return a.second > b.second;});
		
        if(keyFrameCount <= -1 || keyFrameCount >= graph.size()) {
            for(const auto& neigh : connections) {
                CV_Assert(m_frames.count(neigh.first));
                keyframes.push_back(&m_frames.find(neigh.first)->second);
            }
        }
        else {
            CV_Assert(keyFrameCount > 0);
            keyframes.reserve(keyFrameCount);
            std::sort(connections.begin(), connections.end(),
                      [](const std::pair<long, size_t>& a, const std::pair<long, size_t>& b)->bool{return a.second > b.second;});
            
            int weight;
            for(int i=0; i<keyFrameCount; i++) {
                CV_Assert(m_frames.count(connections[i].first));
                keyframes.push_back(&m_frames.find(connections[i].first)->second);

                weight = connections[i].second;
            
            }

            for(int i=keyFrameCount; i<connections.size(); i++) {
                if(connections[i].second == weight) {
                    keyframes.push_back(&m_frames.find(connections[i].first)->second);
                }
            }
        }
        
        return keyframes;
    }
    
    Frame* GlobalMap::findLastFrame(long frameId)
    {
        const long MAX_STEP = 2000;
        //if search backward more than MAX_STEP frame and still failed, we believe there is no more avialble before that.
        for(long i=1; i<MAX_STEP; i++){
            ygomi::Frame* lastFrame = getFrame(frameId - i);
            if(lastFrame) {
                return lastFrame;
            }
        }
        return nullptr;
    }

    // read global map
    ygomi::Frame* GlobalMap::getFrame(long frameId)
    {
        return m_frames.count(frameId) ? &m_frames.find(frameId)->second : nullptr;
    }
    
    ygomi::MapPoint* GlobalMap::getMapPoint(long mapPointId)
    {
        return m_mapPoints.count(mapPointId) ? &(m_mapPoints.find(mapPointId)->second) : nullptr;
    }
	
    size_t find(const std::vector<long>& list, long value)
    {
        const auto& iter = std::find(list.begin(), list.end(), value);
        if(iter != list.end())
            return std::distance(list.begin(), iter);
        size_t idx = 0;
        for(idx=0; idx<list.size(); idx++) {
            if(value < list[idx]) {
                break;
            }
        }

        return idx;
    }

    const std::vector<ygomi::Frame*> GlobalMap::getInterestKeyFrames(long frameId, int keyFrameCount)
    {
        //const auto& iter = std::find(m_keyFramesIndices.begin(), m_keyFramesIndices.end(), frameId);
        //CV_Assert(iter != m_keyFramesIndices.end()); //todo: need current frame is a key frame?
        CV_Assert(keyFrameCount > 0);
        
        std::vector<ygomi::Frame*> keyFrames;
        keyFrames.reserve(keyFrameCount);

        //size_t dist = std::distance(m_keyFramesIndices.begin(), iter);
        size_t dist = find(m_keyFramesIndices, frameId);
        for(size_t id=0; id<dist; id++) {
            ygomi::Frame* keyframe = getFrame(m_keyFramesIndices[dist-1-id]);
            CV_Assert(keyframe);
            keyFrames.push_back(keyframe);
            if(keyFrames.size() == keyFrameCount)
                break;
        }
        
        return keyFrames;
    }
    
    const std::vector<ygomi::MapPoint*> GlobalMap::getMapPointInFrame(long frameId)
    {
        CV_Assert(m_frames.count(frameId));
        std::vector<ygomi::MapPoint*> mps;
        
        const std::vector<ygomi::KeyPoint>& keys = getFrame(frameId)->getKeyPoints();
        for(const auto& key : keys) {
            if(key.m_mapPointId.empty())
                continue;
            for(auto id : key.m_mapPointId) {
                ygomi::MapPoint* mp = getMapPoint(id);
                //the same map point is project to different keys on one frame is not allowed.
                CV_Assert(mp && std::find(mps.begin(), mps.end(), mp) == mps.end());
                
                mps.push_back(mp);
            }
        }
        
        return mps;
    }
    
    const std::vector<const ygomi::Frame*> GlobalMap::getAllKeyFrames()
    {
        std::vector<const ygomi::Frame*> keyFrames;
        keyFrames.reserve(m_keyFramesIndices.size());
        for(auto id : m_keyFramesIndices) {
            keyFrames.push_back(&(m_frames.find(id)->second));
        }

        return keyFrames;
    }
    
    const std::vector<const ygomi::Frame*> GlobalMap::getAllFrames()
    {
        std::vector<const ygomi::Frame*> frames;
        frames.reserve(m_frames.size());
        for(const auto& elem : m_frames) {
            frames.push_back(&(elem.second));
        }
        return frames;
    }
	
    const std::vector<ygomi::MapPoint*> GlobalMap::getAllMapPoints()
    {
        std::vector<ygomi::MapPoint*> mapPoints;
        mapPoints.reserve(m_mapPoints.size());
        for(auto& elem : m_mapPoints) {
            mapPoints.push_back(&elem.second);
        }
        return mapPoints;
    }
    
    const std::list<long>& GlobalMap::getRecentAddedMapPointsIndices() const
    {
        return m_lRecentAddedMapPointsIndices;
    }
    
    const ygomi::Frame* GlobalMap::checkKeyFrames()
    {
        for(auto id : m_keyFramesIndices) {
            const auto& frame = m_frames.find(id);
            if(frame->second.getType() != ygomi::KEY_FRAME)
                return &frame->second;
        }
        
        return nullptr;
    }

    //
    int GlobalMap::getLocalMapPoints(long frameId, std::vector<ygomi::MapPoint *> &plocalMapPoints, std::vector<ygomi::Frame*>& localKeyFrames)
    {
#ifdef RUN_TEST
        //read key frames, map points from file.
        std::vector<long> localKeyFrameIndices;
        readAllLocalKeyFrameIndices(localKFPath, localKeyFrameIndices, frameId);
//        CV_Assert(m_frames.count(frameId));
#else
        CV_Assert(m_frames.count(frameId));
        
        //update local key frames.
        std::unordered_map<long, size_t> keyFrameCounter;
        ygomi::Frame* frame = &m_frames.find(frameId)->second; //update reference key frame.
        for(const auto& key : frame->getKeyPoints()) { //for all key points of current frame.
            if(!key.m_mapPointId.empty()) {
                for(auto id : key.m_mapPointId) { // for each matching pair.
                    CV_Assert(m_mapPoints.count(id));
                    const ygomi::MapPoint* mp = &m_mapPoints.find(id)->second;
                    if(mp->isBad())
                        continue;
                    
                    const std::vector<ygomi::Track>& observations = mp->getObservation();
                    for(const auto& track : observations) {
                        CV_Assert(getFrame(track.m_frameId));
                        
                        if(!keyFrameCounter.count(track.m_frameId))
                            keyFrameCounter[track.m_frameId] = 1; //initial
                        else
                            keyFrameCounter[track.m_frameId]++;
                    }
                }
            }
        }
        
        if(keyFrameCounter.empty())
            return -1;
        
        //select the key frames that current directly could see.
        long maxKFID;
        size_t nmaxCount = 0;
        std::vector<long> localKeyFrameIndices;
        for(const auto& item : keyFrameCounter) {
            long keyframeId = item.first;
            CV_Assert(std::find(m_keyFramesIndices.begin(), m_keyFramesIndices.end(), keyframeId) != m_keyFramesIndices.end());
            const ygomi::Frame* keyframe = getFrame(keyframeId);
            CV_Assert(keyframe);
            if(keyframe->getType() == ygomi::KEY_FRAME) {
                localKeyFrameIndices.push_back(keyframeId); //key frame. filter BAD_KEY_FRAME
            }
            if(nmaxCount < item.second) {
                nmaxCount    = item.second;
                maxKFID      = item.first;
            }
        }
        
        //update reference key frame.
        frame->setRefFrameId(maxKFID);
        
        //
        const int maxLocalKeyFramesNum = 80;
        std::vector<long> keyFrameIndicesChild;
        int newAddCount = 0;
        
        for(auto iter = localKeyFrameIndices.begin(); iter != localKeyFrameIndices.end(); iter++) {
            long keyFrameId = *iter;
            if(localKeyFrameIndices.size()+newAddCount > maxLocalKeyFramesNum)
                break;
            
            int threshold = 10;
            const std::vector<ygomi::Frame*> keyframes = getBestCovisibilityKeyFrames(keyFrameId, threshold);
            for(const auto& keyframe : keyframes) {
                CV_Assert(keyframe);
                if(keyframe->getType() == ygomi::BAD_KEY_FRAME)
                    continue;
                
                // alreadly exist in the list.
                if(std::find(localKeyFrameIndices.begin(), localKeyFrameIndices.end(), keyframe->getId()) != localKeyFrameIndices.end())
                    continue;
                if(std::find(keyFrameIndicesChild.begin(), keyFrameIndicesChild.end(), keyframe->getId()) != keyFrameIndicesChild.end())
                    continue;
                newAddCount++;
                keyFrameIndicesChild.push_back(keyframe->getId());
                
                //break; //???
            }
            
            //todo: search childs and parent.
        }
        
        localKeyFrameIndices.insert(localKeyFrameIndices.end(), keyFrameIndicesChild.begin(), keyFrameIndicesChild.end());
#endif //RUN_TEST

//        std::sort(localKeyFrameIndices.begin(), localKeyFrameIndices.end(), [](long a, long b)->bool{return a < b;});
        
        //update local map points.
        plocalMapPoints.clear();
        for(auto id : localKeyFrameIndices) {
            const ygomi::Frame* keyframe = getFrame(id);
            CV_Assert(keyframe);
            const std::vector<long>& indices = keyframe->getMapPointIndices();
            for(auto mapId : indices) {
                ygomi::MapPoint* mp = getMapPoint(mapId);
                CV_Assert(mp);
                if(mp->isBad()) //bad map pints.
                    continue;
                
                // alreadly exist in the list.
                if(std::find(plocalMapPoints.begin(), plocalMapPoints.end(), mp) != plocalMapPoints.end())
                    continue;

                plocalMapPoints.push_back(mp);
            }
        }
        
        return 0;
    }
    
    static cv::Point3f projectW2C(const cv::Mat& pose3D, const cv::Point3f& Pw)
    {
        CV_Assert(pose3D.type() == CV_64FC1);
        float v[3];
        for(int i=0; i<3; i++) {
            v[i] = pose3D.at<double>(i, 0) * Pw.x +
                   pose3D.at<double>(i, 1) * Pw.y +
                   pose3D.at<double>(i, 2) * Pw.z +
                   pose3D.at<double>(i, 3);
        }
        return cv::Point3f(v[0], v[1], v[2]);
    }
    
    bool GlobalMap::isInFrustum(long frameId, long mapPointId, cv::Mat& m_K, cv::Rect& imgRect, float viewCosThreshold, float mfScaleFactor, int maxLevel)
    {
        CV_Assert(m_frames.count(frameId)&& m_mapPoints.count(mapPointId));
        const ygomi::Frame* frame = &m_frames.find(frameId)->second;
        ygomi::MapPoint* mp = &m_mapPoints.find(mapPointId)->second;
//#define Test_isInFrustum
#ifdef  Test_isInFrustum
        std::cout << "\nmpId= " << mp->getId() << std::endl;
#endif
        //
        mp->m_mbTrackInView = false;

        //Transform from World coordinate to Camera coordinate.
        const cv::Point3f& Pc = projectW2C(frame->getPose(), mp->getPosition());
#ifdef  Test_isInFrustum
        std::cout << "wPosition= " << mp->getPosition() << std::endl;
        std::cout << "cPosition= " << Pc.x << " " << Pc.y << " " << Pc.z << std::endl;
#endif

        //behind camera.
        if(Pc.z < 0.0f)
            return false;

        float fx, fy;
        float cx, cy;
        float min_x, min_y, max_x, max_y;
        float max_dist;
        float min_dist;

        // Need to check m_K's type is CV_32 or CV_64
        fx    = m_K.at<float>(0, 0);
        fy    = m_K.at<float>(1, 1);
        cx    = m_K.at<float>(0, 2);
        cy    = m_K.at<float>(1, 2);

//        fx    = m_K.at<double>(0, 0);
//        fy    = m_K.at<double>(1, 1);
//        cx    = m_K.at<double>(0, 2);
//        cy    = m_K.at<double>(1, 2);

        min_x = 0.0f;
        max_x = imgRect.width;
        min_y = 0.0f;
        max_y = imgRect.height;

//#ifdef  Test_isInFrustum
//        max_dist = param_input[0] * 1.2f;
//        min_dist = param_input[1] * 0.8f;
//#else
        min_dist = mp->getMinDistance();
        max_dist = mp->getMaxDistance();
//#endif
        //project point from camera coordinate to image coordinate.
        float inv_z = 1.0f / Pc.z;
        float x = fx * Pc.x * inv_z + cx;
        float y = fy * Pc.y * inv_z + cy;
#ifdef  Test_isInFrustum
        std::cout << "invz= " << inv_z << std::endl;
        std::cout << "fx, fy, cx, cy= " << fx << " " << fy << " " << cx << " " << cy << std::endl;
        std::cout << "x= " << x << std::endl;
        std::cout << "y= " << y << std::endl;
#endif

        //check boundary
        if(x < min_x || x > max_x || y < min_y || y > max_y) //out of image boundary
            return false;

        //check distance is in the scale invariance region of the map point.
        const cv::Point3f&  PO  = mp->getPosition() - frame->getCameraCenter();
        
        float dist = cv::norm(PO);
#ifdef  Test_isInFrustum
        std::cout << "maxDistance= " << max_dist << std::endl;
        std::cout << "minDistance= " << min_dist << std::endl;
        std::cout << "mnMinX= " << min_x << std::endl;
        std::cout << "mnMaxX= " << max_x << std::endl;
        std::cout << "mnMinY= " << min_y << std::endl;
        std::cout << "mnMaxY= " << max_y << std::endl;
        std::cout << "PO= " << PO << std::endl;
        std::cout << "dist= "  << dist << std::endl;
#endif

        if(dist < min_dist || dist > max_dist)
            return false;



        //check viewing angle.
//#ifdef  Test_isInFrustum
//        cv::Mat Pn =(cv::Mat_<float>(3,1)<<param_input[2], param_input[3], param_input[4]);
//#else
		const cv::Mat& Pn = mp->getNormal();
//#endif
        float dot_result = Pn.at<float>(0, 0) * PO.x + Pn.at<float>(1, 0) * PO.y + Pn.at<float>(2, 0) * PO.z;
        const float viewCos = dot_result / dist;

#ifdef  Test_isInFrustum
        std::cout << "Pn= " << Pn << std::endl;
        std::cout << "viewCos= " << viewCos << std::endl;
#endif
        if(viewCos < viewCosThreshold)
            return false;

        //predict scale in image.
        const int nPredictedLevel  = mp->predictScale(dist, log(mfScaleFactor), maxLevel);
#ifdef  Test_isInFrustum
        std::cout << "nPredictedLevel= " << nPredictedLevel << std::endl;
#endif

        //valid map point for tracking.
        mp->m_mbTrackInView = true;
//        mp->mTrackProjX   = x;
//        mp->mTrackProjY   = y;
        mp->m_mnTrackScaleLevel = nPredictedLevel;
        mp->m_mTrackViewCos = viewCos;
        
        return true;
    }
    
    static inline float dot(const cv::Mat& t, const cv::Point3f& p)
    {
        return t.at<double>() * p.x + t.at<double>() * p.y + t.at<double>() * p.z;
    }
    
    float GlobalMap::computeNthDepth(long frameId, int n)
    {
        const ygomi::Frame* frame = getFrame(frameId);
        CV_Assert(frame);
        
        const cv::Mat& pose3D = frame->getPose();
        cv::Mat R = pose3D.row(2).colRange(0, 3);
        R = R.t();
        float Zcw = R.at<double>(2, 3);
        
        const auto& keys = frame->getKeyPoints();
        std::vector<float> vdepths;
        vdepths.reserve(n);
        for(const auto& key : keys) {
            for(const auto& mpId : key.m_mapPointId) {
                ygomi::MapPoint* mp = getMapPoint(mpId);
                CV_Assert(mp);
                
                const cv::Point3f& x3Dw = mp->getPosition();
                float z = dot(R, x3Dw) + Zcw;
                
                vdepths.push_back(z);
            }
        }
        
        std::sort(vdepths.begin(), vdepths.end());
        
        return vdepths[(vdepths.size()-1)/n];
    }
}
