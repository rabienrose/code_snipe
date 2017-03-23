/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Frame.hpp
 * @brief  Frame is a core structure in SLAM system, each frame input is saved
 *         as a Frame, and is set as a key frame(FrameType) if it pass some 
 *         conditions, also a Frame saves all 2d keys extracted from the image,
 *         and those relations to MapPoint.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once


#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace ygomi
{
    struct KeyPoint;
    
    enum FrameType
    {
        UNKNOWN_TYPE  = 0,
        NORMAL_FRAME  = 1, //normal frame, default type
        KEY_FRAME     = 2, //key frame.
        BAD_KEY_FRAME = 3  //bad key frame.
    };
    
    class Frame
    {
    public:
        Frame(long id);
        Frame(const Frame& frame);
        Frame& operator = (const Frame& frame);
        virtual ~Frame();
        
    public:
        /// Set/Get pairs.
        /**
         *@brief Set type for frame, see the definition of FrameType.
         *@param type Frame type.
         */
        void setType(ygomi::FrameType type);
        
        /**
         *@brief Set camera pose for frame, it's a 3 x 4 matrix in CV_64FC1
         */
        void setPose(const cv::Mat& pose);

        /**
         *@brief Set name for frame.
         */
        void setName(const std::string& name);
        
        /**
         *@brief Set reference frame id of this frame.
         */
        void setRefFrameId(const long refId);
        
        /**
         *@brief Add map point index for single key in frame.
         * @param keyId      Key index in frame.
         * @param mapPointId Map point index will add to 2d key.
         */
        void addMatchingPair(size_t keyId, long mapPointId);

        /**
         *@brief Remove map point index of single key.
         * @param keyId      Key index in frame.
         * @param mapPointId Map point index will be removed.
         */
        void delMatchingPair(size_t keyId, long mapPointId);
        
        /**
         *@brief Clear map point indices of single key.
         * @param keyId Key index in frame.
         */
        void resetMatchingPair(size_t keyId);
        
        /**
         *@brief Add single 2D key to frame.
         * @param key Single 2D key.
         */
        void addKey(const ygomi::KeyPoint& key);

        /**
         *@brief Add multiple 2D keys to frame.
         * @param keys Multiple 2D keys.
         */
        void addKeys(const std::vector<ygomi::KeyPoint>& keys);
        
        /**
         *@brief Clear all keys of frame.
         */
        void resetKeys();
        
        /**
         *@brief Get frame ID.
         */
        long getId() const;
        
        /**
         *@brief Get reference frame id of this frame.
         */
        long getRefFrameId() const;

        /**
         *@brief Get frame type.
         */
        FrameType getType() const;
        
        /**
         *@brief Get frame name.
         */
        const std::string& getName() const;
        
        /**
         *@brief Get all the key points of frame.
         */
        const std::vector<ygomi::KeyPoint>& getKeyPoints() const;
        
        /**
         *@brief Get specified key point of frame.
         */
        const ygomi::KeyPoint& getKeyPoint(size_t keyId) const;
        
        /**
         *@brief Get camera pose of frame.
         */
        const cv::Mat& getPose() const;
        
        /**
         *@brief Get all the map point indices that frame could observe.
         */
        const std::vector<long> getMapPointIndices() const;
        
        /**
         *@brief Get 3D positon of camera center.
         */
        const cv::Point3f& getCameraCenter() const;


        /**
         *@brief compute center distance with prekeyframe.
         */
        double _computeCenterDis2_withPrev(long frameId);


        /**
         *@brief Compute Scene Depth (q=2 median).
         */
        float computeSceneMedianDepth(const std::vector<cv::Point3f> mpPosiSet, const int q) const;
        
    protected:
        long                         m_id;     // Frame id, in general, this information is saved inside the Frame,
                                               // but in our case, we need to debug or search the history info, so we
                                               // ask caller to set this id, and MAKE SURE it is distinguishing.
        long                         m_refId;  // reference frame id of this frame.
        FrameType                    m_type;   // Frame type.
        std::string                  m_name;   // Image name in disk.
        std::vector<ygomi::KeyPoint> m_keys;   // 2D key points in frame.
        cv::Mat                      m_pose;   // 3D pose of frame, 3 x 4 (R | t) matrix.
        cv::Point3f                  m_cameraCenter;    // 3D positon of camera center.
    };
}