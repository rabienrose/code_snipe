/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MapPoint.cpp
 * @brief  Implementation of class MapPoint
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#include "MapPoint.hpp"
#include "TypeDef.hpp"
#include "FeatureTool.hpp"

namespace ygomi {
    
    long MapPoint::m_nextId = 0;
    
    MapPoint::MapPoint(long frameID) :
                        m_isBad(true),
                        m_pt(cv::Point3f(NAN, NAN, NAN)),
                        m_referKFID(frameID),
                        m_firstKFID(frameID),
                        m_minDistance(0.f),
                        m_maxDistance(0.f),
                        m_mbTrackInView(false),
                        m_mnTrackScaleLevel(0),
                        m_mTrackViewCos(0.f),
                        m_nVisibile(1), // To construct a map point, there must be 2 frames, todo:check
                        m_nRealFound(1)
    {
        m_id = m_nextId++;
        m_descriptor.release();
        m_tracks.clear();
        m_normalVector.release();
    }
    
    void MapPoint::operator = (const ygomi::MapPoint& mp)
    {        
        m_isBad      = mp.isBad();
        m_id         = mp.getId();
        m_pt         = mp.getPosition();
        m_descriptor = mp.getDescriptor();
        m_tracks     = mp.getObservation();
    }
    
    MapPoint::~MapPoint()
    {
        m_descriptor.release();
        m_tracks.clear();        
    }
    
    bool MapPoint::isBad() const
    {
        return m_isBad;
    }

    long MapPoint::getId() const
    {
        return m_id;
    }
    
    long MapPoint::getFirstKeyFrameID() const
    {
        return m_firstKFID;
    }
    
    long MapPoint::getReferenceKeyFrameID() const
    {
        return m_referKFID;
    }
    
    const cv::Point3f& MapPoint::getPosition() const
    {
        return m_pt;
    }
    
    const cv::Mat& MapPoint::getDescriptor() const
    {
        return m_descriptor;
    }
    
    const std::vector<ygomi::Track>& MapPoint::getObservation() const
    {
        return m_tracks;
    }
	
	bool MapPoint::getSingleObservation(long frameId, ygomi::Track &track)
	{
		int count = 0;
		for(const auto& t : m_tracks) {
			if(t.m_frameId == frameId) {
				track = t;
				count++;
			}
		}
		
		return count > 0;
	}
	
    bool MapPoint::isInKeyFrame(long frameId) const
    {
        for(const auto& track : m_tracks) {
            if(track.m_frameId == frameId)
                return true;
        }
        
        return false;
    }
    
    float MapPoint::getFoundRatio() const
    {
        return static_cast<float>(m_nRealFound) / m_nVisibile;
    }
    
    // Set handle.
    bool MapPoint::addTrack(long frameId, size_t keyId)
    {
        return addTrack(ygomi::Track(frameId, keyId));
    }

    bool MapPoint::addTrack(const ygomi::Track& track)
    {
        for(const auto& it : m_tracks) {
            if(it.m_frameId == track.m_frameId)
                return false;
        }
        m_tracks.push_back(track);
 
        return true;
    }
    
    void MapPoint::addTracks(const std::vector<ygomi::Track> &tracks)
    {
        for(const auto& it : tracks)
            addTrack(it);
    }
    
    int MapPoint::removeTrack(const ygomi::Track& track)
    {
        int count = 0;
        for(auto it = m_tracks.begin(); it != m_tracks.end(); it++) {
            if(it->m_frameId == track.m_frameId && it->m_keyPointId == track.m_keyPointId) {
                m_tracks.erase(it);
                count++;
                if(it == m_tracks.end())
                    break;
            }
        }
        
        //check reference keyframe.
        if(count > 0 && m_referKFID == track.m_frameId && !m_tracks.empty()) {
            m_referKFID = m_tracks.begin()->m_frameId;
        }
        
        return count;
    }
	
	int MapPoint::removeTrack(long frameId)
	{
		int count = 0;
		auto it = m_tracks.begin();
		while(it != m_tracks.end()) {
			if(it->m_frameId != frameId) {
				it++;
			}
			else {
				it = m_tracks.erase(it);
				count++;
			}
		}
		return count;
	}
    
    void MapPoint::removeAllTracks()
    {
        m_tracks.clear();
        m_referKFID = -1;
    }
    
    
    void MapPoint::setId(long id)
    {
        m_id     = id;
        m_nextId = m_id + 1;
    }
    
    void MapPoint::setFirstKeyFrameID(long frameId)
    {
        m_firstKFID = frameId;
    }
    
    void MapPoint::setReferenceKeyFrameID(long frameId)
    {
        m_referKFID = frameId;
    }

    void MapPoint::setBad(bool isBad)
    {
        m_isBad = isBad;
    }

    void MapPoint::setTrackInView(bool mbTrackInView)
    {
        m_mbTrackInView = mbTrackInView;
    }

    void MapPoint::setTrackViewCos(float mTrackViewCos)
    {
        m_mTrackViewCos = mTrackViewCos;
    }

    void MapPoint::setTrackScaleLevel(int mnTrackScaleLevel)
    {
        m_mnTrackScaleLevel = mnTrackScaleLevel;
    }


    void MapPoint::setPosition(const cv::Point3f& pt)
    {
        m_pt = pt;
    }
    
    bool MapPoint::mergeDescriptor(const std::vector<cv::Mat>& descriptors)
    {
        pDistFunc calcDistfunc;
        switch (ygomi::KeyPoint::m_descType) {
            case ygomi::BINARY_DESC:
                calcDistfunc = calcDistanceHamming;
                break;
            case ygomi::FLOAT_DESC:
                calcDistfunc = calcDistanceEuler;
                break;
            default:
                calcDistfunc = calcDistanceEuler;
                break;
        }
        
        CV_Assert(descriptors.size() == m_tracks.size());
        const size_t N = m_tracks.size();
        if(!N) return false;
        
        float distances[N][N];
        for(size_t i=0;i<N;i++)
        {
            distances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                float distij = calcDistfunc(descriptors[i], descriptors[j]);
                //std::cout<<distij<<std::endl;
                distances[i][j]=distij;
                distances[j][i]=distij;
            }
        }
        
        //todo: double-check the distance type.
        int bestMedian = INT_MAX;
        int bestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            std::vector<int> vdists(distances[i],distances[i]+N);
            sort(vdists.begin(),vdists.end());
            int median = vdists[0.5*(N-1)];
            
            if(median<bestMedian)
            {
                bestMedian = median;
                bestIdx = i;
            }
        }
        
        m_descriptor = descriptors[bestIdx].clone();
        return true;
    }
    
    void MapPoint::setDescriptor(const cv::Mat &descriptor)
    {
        m_descriptor = descriptor.clone();        
    }
    
    bool MapPoint::isValidReferenceKFID(size_t& referKFIndexInTrack)
    {
        bool isvalid = false;
        for(size_t i=0; i<m_tracks.size(); i++) {
            if(m_tracks[i].m_frameId == m_referKFID) {
                referKFIndexInTrack = i;
                isvalid             = true;
                break;
            }
        }
        
        return isvalid;
    }
    
    void MapPoint::updateNormalAndDepth(const std::vector<cv::Point3f> cameraCenters, const int level, const std::vector<float> scaleFactorSet)
    {
        if (m_isBad || m_tracks.empty()) {
            return;
        }
        
        CV_Assert(cameraCenters.size() == m_tracks.size());
        size_t referKFIndex = 0;
        CV_Assert(isValidReferenceKFID(referKFIndex));
        
        cv::Mat mpPosi(3, 1, CV_32FC1);
        mpPosi.at<float>(0, 0) = m_pt.x;
        mpPosi.at<float>(1, 0) = m_pt.y;
        mpPosi.at<float>(2, 0) = m_pt.z;
        
        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int num        = cameraCenters.size();
        for (int i = 0; i < num; i++) {
            cv::Mat cCw(3, 1, CV_32FC1);
            cCw.at<float>(0, 0) = cameraCenters[i].x;
            cCw.at<float>(1, 0) = cameraCenters[i].y;
            cCw.at<float>(2, 0) = cameraCenters[i].z;
            
            cv::Mat normali = mpPosi - cCw;
            normal = normal + normali / cv::norm(normali);
        }
        m_normalVector = normal / num;

        //calculate min-max distance of mappoint.
        cv::Mat cCwRef(3, 1, CV_32FC1);
        cCwRef.at<float>(0, 0) = cameraCenters[referKFIndex].x;
        cCwRef.at<float>(1, 0) = cameraCenters[referKFIndex].y;
        cCwRef.at<float>(2, 0) = cameraCenters[referKFIndex].z;
        cv::Mat PC                   = mpPosi - cCwRef;
        const float dist             = cv::norm(PC);
        const float levelScaleFactor = scaleFactorSet[level];
        const int nlevels            = scaleFactorSet.size();
        
        m_maxDistance = dist * levelScaleFactor;
        m_minDistance = m_maxDistance / scaleFactorSet[nlevels - 1];
    }
    
    void MapPoint::addVisible(int n)
    {
        m_nVisibile += n;
    }
    
    void MapPoint::addFound(int n)
    {
        m_nRealFound += n;
    }
    
    int MapPoint::predictScale(float currDistance, float logScaleFactor, int nMaxLevel)
    {
        float ratio = m_maxDistance / currDistance;

        int scl = static_cast<int>(ceil(log(ratio)/logScaleFactor));
        scl = scl < 0 ? 0 : scl;
        scl = scl > nMaxLevel - 1 ? nMaxLevel - 1 : scl;
        
        return scl;
    }
}
