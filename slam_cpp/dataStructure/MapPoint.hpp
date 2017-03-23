/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   MapPoint.hpp
 * @brief  MapPoint is a core structure in SLAM system, including position in 3D,
 *         descriptor and it's track.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.07.29        Qiang.Zhai     Create
 *******************************************************************************
 */
#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace ygomi
{
    
    struct Track;
    
    class MapPoint
    {
    public:
        MapPoint(long frameID);
        ~MapPoint();
        
        void operator = (const ygomi::MapPoint& mp);
    public:
        // Get handle.
        /**
         *@brief Check whether map point is bad.
         * return true if map point is bad, false otherwise.
         */
        bool isBad() const;
        
        /**
         *@brief Get ID of map point.
         */
        long getId() const;
        
        /**
         *@brief //todo
         */
        long getFirstKeyFrameID() const;
        
        /**
         *@brief //todo
         */
        long getReferenceKeyFrameID() const;
        
        /**
         *@brief Get map point position in world coordinate.
         */
        const cv::Point3f& getPosition() const;
    
        /**
         *@brief Get descriptor of map point.
         */
        const cv::Mat& getDescriptor() const;
        
        /**
         *@brief Get observations of map point.
         */
        const std::vector<ygomi::Track>& getObservation() const;
		
		/**
		 *@brief Get specified track
		 *@param frameId
		 *@param track	 [output]
		 *@return		 true if find a track.
		 */
		bool getSingleObservation(long frameId, ygomi::Track& track);
		
        /**
         *@brief //todo
         */
        float getFoundRatio() const;
        
        /**
         *
         */
        int getVisible() const { return m_nVisibile; }
        
        /**
         *
         */
        int getRealFould() const { return m_nRealFound; }
        
        /**
         *@brief //todo
         *NOTE!!!!move the constant value out!!!!!
         */
        inline float getMinDistance() const { return 0.8f * m_minDistance; }
        
        /**
         *
         */
        inline float getMaxDistance() const { return 1.2f * m_maxDistance; }
        
        /**
         *
         */
        const cv::Mat& getNormal() const { return m_normalVector; }
        
        /**
         *@brief Check whether map point is in specified key frame.
         */
        bool isInKeyFrame(long frameId) const;
        
        // Set handle.
        /**
         *@brief Add a observation to map point.
         * @param frameId Which frame that map point can observe.
         * @param keyId   Which key in frame map point correspond.
         * return true if success, false otherwise.
         */
        bool addTrack(long frameId, size_t keyId);

        /**
         *@brief Add a observation to map point.
         * @param track Observation is added to map point.
         * return true if success, false otherwise.
         */
        bool addTrack(const ygomi::Track& track);
        
        /**
         *@brief Add multi-observations to map point.
         * @param tracks Multiple observations are added to map point.
         */
        void addTracks(const std::vector<ygomi::Track>& tracks);
        
        /**
         *@brief Remove specify observation of map point.
         * @param track Observation will be moved.
         * return number of observations just removed.
         */
        int removeTrack(const ygomi::Track& track);
		
		/**
		 *@brief Reove specify observation of map point, only by frameId.
		 * @param frameId Observation will be removed.
		 * @return		  number of observations just removed.
		 */
		int removeTrack(long frameId);
		
        /**
         *@brief Clear all the observations of map point.
         */
        void removeAllTracks();
        
        /**
         *@brief Set map point id.
         *        DO NOT call this interface if you don't know what this mean.
         * @param id Map point ID.
         */
        void setId(long id);

        /**
         *
         */
        void  setTrackInView(bool mbTrackInView);

        /**
         *
         */
        void  setTrackViewCos(float mTrackViewCos);

        /**
         *
         */
        void  setTrackScaleLevel(int mnTrackScaleLevel);

        /**
         *@brief //todo
         */
        void setFirstKeyFrameID(long frameId);

        /**
         *@brief //todo
         */
        void setReferenceKeyFrameID(long frameId);

        /**
         *@brief Set map point to bad.
         * @param isBad Status identifier, set true as default.
         */
        void setBad(bool isBad = true);
        
        /**
         *@brief Set map point position in World Coordinate.
         */
        void setPosition(const cv::Point3f& pt);
        
        /**
         *@brief Compute descriptor by observations of map point.
         * @param descriptors Descriptors set of observations.
         * return true if success, false otherwise.
         */
        bool mergeDescriptor(const std::vector<cv::Mat>& descriptors);
        
        /**
         *@brief Set descriptor to map point.
         */
        void setDescriptor(const cv::Mat& descriptor);
        
        /**
         *@brief Update mean viewing direction and scale invariance distances.
         */
        void updateNormalAndDepth(const std::vector<cv::Point3f> cameraCenters, const int level, const std::vector<float> scaleFactorSet);
        
        /**
         * //todo
         */
        void addVisible(int n);
        
        /**
         * //todo
         */
        void addFound(int n);
        
        /**
         *
         */
        int predictScale(float currDistance, float logScaleFactor, int nMaxLevel);
        
        //after clear the GlobalMap. This var should be set to 0 also
        static long               m_nextId;     // Next map point id.
       
    protected:
        bool isValidReferenceKFID(size_t& referKFIndexInTrack);
        
    protected:
        ///Intrinsic Properties
        bool                      m_isBad;          // Map point status, if it is set to true, current map point is discarded.
        long                      m_id;             // Map point id.
        cv::Point3f               m_pt;             // Map point position.
        cv::Mat                   m_descriptor;     // Map point descriptor, a merged descriptor from all tracks.
        std::vector<ygomi::Track> m_tracks;         // Each map point(3D) is projected into multiple key points(2D),
                                                    // those key points list is defined as the track of current map point.
    public: //todo: make this protected.
        ///Other assisted properties.
        long                      m_referKFID;
        long                      m_firstKFID;      // The first index of key frame construct map point.
        float                      m_minDistance;    // Scale invariance distances
        float                      m_maxDistance;
        cv::Mat                  m_normalVector;   // Mean viewing direction
        // Variables used by the tracking
        bool                      m_mbTrackInView;
        int                         m_mnTrackScaleLevel;
        float                      m_mTrackViewCos;
        int                         m_nLastFrameSeen;

    protected:
        //Tracking counters
        int                       m_nVisibile;      // Frame count should be found.
        int                       m_nRealFound;     // Frame count actual found.
    };
}
