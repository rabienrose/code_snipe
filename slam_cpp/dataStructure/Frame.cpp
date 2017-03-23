/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Frame.cpp
 * @brief  Implementation of class Frame
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#include "Frame.hpp"
#include <cassert>

#include "TypeDef.hpp"

namespace ygomi {
    
    Frame::Frame(long id) :
        m_id(id),
        m_refId(id),
        m_type(UNKNOWN_TYPE),
        m_name("")
    {
        m_keys.clear();
        m_pose.release();
        m_cameraCenter = cv::Point3f(NAN, NAN, NAN);
    }
    
    Frame::Frame(const Frame& frame) :
        m_id(frame.getId()),
        m_refId(frame.getRefFrameId()),
        m_type(frame.getType()),
        m_name(frame.getName()),
        m_cameraCenter(frame.getCameraCenter())
    {
        m_keys = frame.getKeyPoints();
        m_pose = frame.getPose();
    }
    
    Frame& Frame::operator = (const ygomi::Frame &frame)
    {
        m_id            = frame.getId();
        m_refId         = frame.getRefFrameId();
        m_type          = frame.getType();
        m_name          = frame.getName();
        m_keys          = frame.getKeyPoints();
        m_pose          = frame.getPose();
        m_cameraCenter  = frame.getCameraCenter();
        
        return *this;
    }
    
    Frame::~Frame()
    {
        m_keys.clear();
    }
    
    void Frame::setType(ygomi::FrameType type)
    {
        m_type = type;
    }
    
    void Frame::setPose(const cv::Mat& pose)
    {
        CV_Assert(pose.type() == CV_64FC1);
        m_pose = pose.clone();
        
        cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3);
        cv::Mat twc = pose.rowRange(0, 3).col(3);
        cv::Mat Ow = -Rwc.t() * twc; // to be confirmed
        
        m_cameraCenter.x = Ow.at<double>(0, 0);
        m_cameraCenter.y = Ow.at<double>(1, 0);
        m_cameraCenter.z = Ow.at<double>(2, 0);
        
//        std::cout << "m_cameraCenter " << m_cameraCenter << std::endl;
//        std::cout << "pose " << pose << std::endl;
//        std::cout << "Rwc " << Rwc << std::endl;
//        std::cout << "twc " << tcw << std::endl;
//        std::cout << "Ow " << Ow << std::endl;
    }

    void Frame::setName(const std::string& name)
    {
        m_name = name;
    }
    
    void Frame::setRefFrameId(const long refId)
    {
        m_refId = refId;
    }
    
    void Frame::addMatchingPair(size_t keyId, long mapPointId)
    {
        CV_Assert(keyId >= 0 && keyId < m_keys.size());
        std::vector<long>& mapPointIdSet = m_keys[keyId].m_mapPointId;
        if(std::find(mapPointIdSet.begin(), mapPointIdSet.end(), mapPointId) == mapPointIdSet.end()) {
            mapPointIdSet.push_back(mapPointId);
        }
    }
    
    void Frame::delMatchingPair(size_t keyId, long mapPointId)
    {
        CV_Assert(keyId >= 0 && keyId < m_keys.size());
        std::vector<long>& mpList = m_keys[keyId].m_mapPointId;
        for (std::vector<long>::iterator it=mpList.begin(); it!=mpList.end(); it++){
            if (*it == mapPointId){
                mpList.erase(it);
                break;
            }
        }
    }
   
    void Frame::resetMatchingPair(size_t keyId)
    {
        CV_Assert(keyId >= 0 && keyId < m_keys.size());
        m_keys[keyId].m_mapPointId.clear();
    }
    
    void Frame::addKey(const ygomi::KeyPoint& key)
    {
        m_keys.push_back(key);
    }
    
    void Frame::addKeys(const std::vector<ygomi::KeyPoint>& keys)
    {
        m_keys.reserve(m_keys.size() + keys.size());
        for(const auto& key : keys) {
            addKey(key);
        }
    }

    void Frame::resetKeys()
    {
        m_keys.clear();
    }
    
    long Frame::getId() const
    {
        return m_id;
    }
    
    long Frame::getRefFrameId() const
    {
        return m_refId;
    }
    
    FrameType Frame::getType() const
    {
        return m_type;
    }
    
    const std::string& Frame::getName() const
    {
        return m_name;
    }
    
    const std::vector<ygomi::KeyPoint>& Frame::getKeyPoints() const
    {
        return m_keys;
    }
    
    const ygomi::KeyPoint& Frame::getKeyPoint(size_t keyId) const
    {
        CV_Assert(keyId >= 0 && keyId < m_keys.size());
        return m_keys[keyId];
    }
    
    const cv::Mat& Frame::getPose() const
    {
        return m_pose;
    }
    
    const std::vector<long> Frame::getMapPointIndices() const
    {
        std::vector<long> indices;
        for(const auto& key : m_keys) {
            for(auto id : key.m_mapPointId) {
				CV_Assert(std::find(indices.begin(), indices.end(), id) == indices.end());
                indices.push_back(id);
            }
        }
        
        return indices;
    }
    
    const cv::Point3f& Frame::getCameraCenter() const
    {
        return m_cameraCenter;
    }
    
    float Frame::computeSceneMedianDepth(const std::vector<cv::Point3f> mpPosiSet, const int q) const
    {
        int N = mpPosiSet.size();
        std::vector<float> vDepths;
        vDepths.reserve(N);
        cv::Mat Rwc2 = m_pose.row(2).colRange(0, 3);
//        std::cout << "Rcw2: " << Rcw2 << Rcw2.type() << "\n";
        cv::Mat Rcw2 = Rwc2.t();
        Rcw2.convertTo(Rcw2, CV_32FC1);
//        std::cout << "Rcw2(t): " << Rcw2 << "\n";
        float zcw = m_pose.at<float>(2, 3);
//        std::cout << "zcw: " << zcw << "\n";
        for(int i = 0; i < N; i++)
        {
            cv::Mat x3Dw(3, 1, CV_32FC1);
            x3Dw.at<float>(0, 0) = mpPosiSet[i].x;
            x3Dw.at<float>(0, 1) = mpPosiSet[i].y;
            x3Dw.at<float>(0, 2) = mpPosiSet[i].z;
//            std::cout << "x3Dw: " << x3Dw << x3Dw.type() << "\n";
            
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
        
        sort(vDepths.begin(), vDepths.end());
        
        return vDepths[(vDepths.size() - 1) / q];
    }
}
