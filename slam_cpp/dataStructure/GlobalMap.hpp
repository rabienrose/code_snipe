/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   GlobalMap.hpp
 * @brief  Global map in the whole SLAM system, including key frames, map points
 *         and the covibisility infomation between them.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <unordered_map>
#include <map>
#include <vector>
#include <list>
#include "Frame.hpp"
#include "MapPoint.hpp"

namespace ygomi
{
    class  Frame;
    class  MapPoint;
    struct LocalMap;
    
    /**
     *@brief CoreData is a data structre which is used during all the SLAM life cycle.
     */
    class GlobalMap
    {
    public:
        GlobalMap();
        virtual ~GlobalMap();
        
    public:
        /// Read/Write handle
        /**
         *@brief Read global map from file.
         */
        void read(const char* path);

        /**
         *@brief Write global map to file.
         */
        void write(const char* path) const;
        
        /**
         *@brief Clear all the information of global map, including frames, map points,
         *       key frames, etc.
         */
        void reset();

        /// write global map
        /**
         *@brief Add frame to global map, if frame is exist, udpate frame using
         *       Update().
         *  @param frame Added frame.
         */
        void addFrame(const ygomi::Frame& frame);

        /**
         *@brief Construct a new frame by frame ID, add it to global map, and 
         *       return the pointer.
         * @param frameId ID to construct a new frame.
         * return the pointer of frame just constructed.
         */
        Frame* addFrame(long frameId);
        
        /**
         *@brief Update specified frame.
         *       Make sure frame is exist in global map.
         */
        void updateFrame(const ygomi::Frame& frame);
        
        /**
         *@brief Add map point to global map.
         *       Make sure map point is not exist in global map.
         */
        void addMapPoint(const ygomi::MapPoint& mapPoint);
        
        /**
         *@brief Remove the specify map points.
         *       Make sure map point is exist in global map.
         */
        void removeMapPoint(long mapPointId);
        
        /**
         *@brief Add key frame index into key frame indices.
         *       Make sure frame is exist and must be a key frame.
         */
        void addKeyFrame(long frameId);
        
        /**
         *@brief //todo
         */
        void addIndicesToRecentAddedMapPointsList(long mpId);
        
        /**
         *@brief Udpate covisibility gragh according to the observations of map points.
         * @param frameId Update the covibisility graph of specified key frame.
         *                if frameId is not set(-1 as default), update all covisibility graph.
         */
        void updateCovisibilityGraph(long frameId = -1);

        /**
         *@brief Check all the key frames.
         */
        void updateAllKeyFrames();
        
        /**
         *@brief Get the best N key frames connected to a specified key frame in covisibility graph.
         * @param frameId       The specified key frame.
         * @param keyFrameCount Best number key frame connected to the specified key frame.
         * @return              Neighboors connected the specified key frame.
         */
        std::vector<ygomi::Frame*> getBestCovisibilityKeyFrames(long frameId, int keyFrameCount);
        
        /**
         *@brief Get last frame according to the specified frame.
         * @param frameId Frame ID of interested frame.
         * @return The last frame of the spicified frame,
         *         nullptr if not found.
         */
        Frame* findLastFrame(long frameId);
        
        /// read global map
        /**
         *@brief Get frame by frame ID.
         */
        ygomi::Frame* getFrame(long frameId);
        
        /**
         *@brief Get map point by map point ID.
         */
        ygomi::MapPoint* getMapPoint(long mapPointId);
        
        /**
         *@brief Get key frames count backward from the specified frame.
         *       Not including the specified frame.
         */
        const std::vector<ygomi::Frame*> getInterestKeyFrames(long frameId, int keyFrameCount);
        
        /**
         *@brief Get all map points of the interested frame.
         */
        const std::vector<ygomi::MapPoint*> getMapPointInFrame(long frameId);

        /**
         *@brief Get all the key frames in global map(Read Only).
         */
        const std::vector<const ygomi::Frame*> getAllKeyFrames();
                
        /**
         *@brief Get all frames in global map(Read only).
         */
        const std::vector<const ygomi::Frame*> getAllFrames();
        
        /**
         *@brief Get all map points in global map(Read only).
         */
        const std::vector<ygomi::MapPoint*> getAllMapPoints();
        
        /**
         *@brief //todo
         */
        const std::list<long>& getRecentAddedMapPointsIndices() const;
        
        /**
         *@brief    Check the type and other properties of all key frames.
         * @return  The first in-correct key frame, nullptr if all key frames are correct.
         */
        const ygomi::Frame* checkKeyFrames();

		

        /**
         *@brief Check whether map point is inside the vision frustum of frame.
         * @param frameId    Frame ID to specify the frame.
         * @param mapPointId Map pint ID to specify the map point.
         * @return true if the map point is inside the frustum, false otherwise.
         */
        bool isInFrustum(long frameId, long mapPointId, cv::Mat& m_K, cv::Rect& imgRect, float viewCosThreshold, float mfScaleFactor, int maxLevel);

        /**
         *@brief Get local map points of interesting frame by covisibility graph.
         * @param frameId         Frame ID to specify the frame.
         * @param plocalMapPoints [output] Local map points of frame.
         * @return                0 if success,
         *                        -1 if there is none key frame has the covisibility info with frame.
         */
        int getLocalMapPoints(long frameId, std::vector<ygomi::MapPoint*>& plocalMapPoints, std::vector<ygomi::Frame*>& localKeyFrames);
        
        /**
         *@brief Compute Secne Depth(n=2, median) of frame.
         * @param frameId Frame ID to specify the frame.
         * @param n       Number to specify the partition strategy, if n=2, median depth is returned.
         * @return        The n-th depth.
         */
        float computeNthDepth(long frameId, int n);
        
    protected:
        
        void clear();
        
        void updateSingleCovisibilityGraph(long frameId);
        
    protected:
        // All processed frames, each key-value is frame id, double-check it with the id in Frame.
        std::unordered_map<long, ygomi::Frame>                  m_frames;
        
        // All map points, each key-value is map point id, double-check it with the id in MapPoint.
        std::unordered_map<long, ygomi::MapPoint>               m_mapPoints;
        
        // All key frames indices, each value is the index in m_frames, double-check it's frame type.
        std::vector<long>                                       m_keyFramesIndices;
        
        // Covisibility graph of all key frames, each key-value is frame id, double-check it with the id in KeyFrame.
        std::unordered_map<long, std::unordered_map<long, size_t> > m_covisibilityGraph;
        
        // Recent added map points are processed before local BA in mapping.
        std::list<long>                                         m_lRecentAddedMapPointsIndices;
    };
}