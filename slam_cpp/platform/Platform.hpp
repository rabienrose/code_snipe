/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Platform.hpp
 * @brief  Platform is defined as a controller of all the SLAM system, it mainly
 *         includes modules like Tracking, Mapping, I/O, Visualization, etc.
 *         NOTE!!! In order to combine the goodness of Matlab, the platform
 *         offers to Matlab lots of detailing interface used in Tracking and
 *         Mapping, see the comment bellow, so if you do not know what those
 *         interface mean, DO NOT call them.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.03        Qiang.Zhai     Create
 *******************************************************************************
 */

#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <memory>
#include "typeDef.h"
#include "outPut.h"

using namespace roadDBCore;

struct mxArray_tag;

namespace chamo {
    class ViewInterfaceImp;
    class DataInterface;
}

namespace ORB_SLAM2 {
    class ORBextractor;
}

namespace pangolin {
    class OpenGlRenderState;
    class View;
}
namespace algo {
    class Mean;
	class IGps;
}

namespace CommonAlgo {
	class transformTools;
}


namespace ygomi
{
    class GlobalMap;
    class FeatureExtractor;
    class MapPoint;
    class Frame;
    class ParamParser;
    class Mean;
    struct Track;
	
	
	enum TrackingState {
		NOT_INITIALIZED		= 0,
		INITIALIZED 		= 1,
		TRACKING_SUCCESS	= 2,
		TRACKING_LOST		= 3
	};
	
    class Platform
    {
    protected:
        Platform();
    public:
        ~Platform();
        
        ///This must be called firstly to create and initialize the platform.
    public:
        
        /**
         *@brief Create working instance.
         */
        static ygomi::Platform* createInstance();
        
        /**
         *@brief Release working instance, in c plus plus, release job would be
         *       complete automatical, this interface is particular for Matlab users.
         */
        static void releaseInstance();
        
        /**
         *@brief Initialize the working platform by configuration file.
         * @param configPath Configuration file path.
         */
        bool init(const char* configPath, chamo::DataInterface* _dataInterface);
        
        /**
         *@brief Change the camera parameters.
         *       NOTE!!! This is must be called after init.
         * @param fx and fy Focal length by in pixels.
         * @param cx and cy Principal point.
         */
        void changeCameraParam(float fx, float fy, float cx, float cy);
        
        bool processFrame(long frameId, std::string frameName);
        
        ///For general users, these interfaces are enough for a SLAM system.
    public:
        /**
         *@brief Set image to process.
         * @param frameId Current frame ID.
         * @param path    Image path to load image.
         */
        bool setFrame(long frameId, const char* path);
        
        /**
         *@brief Track current frame.
         * @param frameId Current frame ID.
         */
        bool track(long frameId);
        
        /**
         *@brief Do mapping using current frame.
         * @param frameId Current frame ID.
         */
        void map(long frameId);
        
        ///Interface for I/O, and bridges between C and Matlab.
    public:
        /**
         *@brief Inite pose by optical flow.
         * @param frameId Frame ID to specify frame need to process.
         */
        bool tryInitPose(long frameId);
        
        /**
         *@brief Inite map information(eg. mean viewing direction, scale invariance distances, median depth, scale).
         * @param frame1 inital frame.
         * @param frame2 current frame.
         */
        bool initialMap(Frame* frame1, Frame* frame2);
        
        /**
         *@brief Extract keys and descriptors of specified frame.
         * @param frameId Frame ID to specify frame need to process.
         * @param frame   Processed frame.
         */
        void extractFeatures(long frameId);
        
        /**
         *@brief Predict a coarse pose of specified frame.
         * @param frameId Frame ID to specify frame need to process.
         */
        void predictPose(long frameId);
        
        /**
         *@brierf Search 3D-2D matching pairs by projection.
         * @param frameId       Frame ID to specify frame need to process.
         * @param keyFrameCount Number of key frames used to select map points.
         * @param th            threshold
         * @param isImproveTh   if need to relax the threshold
         */
        void searchByProjection(long frameId,
                                int keyFrameCount,
                                float searchRadius,
                                bool isImproveTh,
                                std::vector<ygomi::MapPoint*>& mapPoints,
                                std::vector<std::pair<long, size_t> > &matchingPairs);

        /**
         *@brierf Track with motion model.
         * @param frameId       Frame ID to specify frame need to process.
         */
        bool trackWithMotionModel(long frameId);

        /**
         *@brierf Compute the distance of two descriptors.
         */
        float DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        /**
         *@brierf Search by Brute-force match, not use BoW for SGD.
         * @param refKeyFrame   Reference Key Frame need to process.         
         * @param currFrame		Frame need to process.
         * @param mapPoints     map points of Reference key frame
         * @param matchingPairs Map point matches vector
         */
        void searchByBoW(ygomi::Frame* refKeyFrame,
                         ygomi::Frame* currFrame,
                         const std::vector<ygomi::MapPoint*>& mapPoints,
                         std::vector<std::pair<size_t, size_t> > &matchingPairs);

        /**
         *@brierf Track Reference Key Frame.
     	 * @param frameId       Frame ID to specify frame need to process.
         */
        bool trackReferenceKeyFrame(long frameId);

        /**
         *@brief Optimize pose of specified frame.
         * @param frameId Frame ID to specify frame need to process.
         */
        bool optimizePose(long frameId);
        
        /**
         *@brief Track local map of specified frame.
         * @param frameId Frame ID to specify frame need to process.
         */
        bool trackLocalMap(long frameId);
        
        /**
         *@brief Dicide whether the specified frame is a key frame.
         * @param frameId Frame ID to specify frame need to process.
         */
        void decisionKeyFrame(long frameId);
        
        /**
         *@brief Find 2D-2D matching pairs by epipolar.
         * @param frameId       Frame ID to specify frame need to process.
         * @param keyFrameCount Number of key frames used to select un-triangulated keys.
         */
        void searchByEpipolar(long frameId, int keyFrameCount);

        /**
         *@brief Triangulate 2D-2D matching pairs to map points(3D).
         * @param frameId       Frame ID to specify frame need to process.
         * @param keyFrameCount Number of key frames used to select 2D-2D matching pairs.
         */
        void triangulate(long frameId, int keyFrameCount);
        
        void createNewMapPoints(long frameId, int keyFrameCount);
        
        /**
         *@brief Search 3D-2D matching pairs by projection back(from current key frame to the last ones).
         * @param frameId       Frame ID to specify frame need to process.
         * @param keyFrameCount Number of key frames used to select map points.
         */
        void searchByProjectionBack(long frameId, int keyFrameCount);
        
        /**
         *@brief Cull map points in global map.
         * @param frameId       Frame ID to specify frame need to process.
         */
        void cullMapPoint(long frameId);
        
        /**
         *@brief Fuse map points in global map.
         * @param frameId       Frame ID to specify frame need to process.
         */
        void fuseMapPoint(long frameId);
		
		/**
		 *
		 *
		 */
		void optimizePoseByLocalBA(long frameId);
		
        /**
         *@brief Optimize local map by Bandule Adjustment.
         * @param frameId       Frame ID to specify frame need to process.
         * @param keyFrameCount Number of key frames used to optimize.
         */
        void localBA(long frameId, int keyFrameCount);


        /**
         *@brief compute center distance with previous key frame.
         */
        double computeCenterDis2_withPrev(long frameId);

        /**
         *@brief Cull key frame in global map.
         */
        void cullKeyFrame(long frameId);
        
        /**
         *@Use opticflow to match the kp of two frames
         */
        bool MatchByOpticFlow(long i, int offset);
        
        GlobalMap* getGMapPtr();
        
        //Function to save data to visualization platfrom
        void AddActMpToVisPlat(int channel, int mpId, cv::Mat posi, cv::Mat color);
        void SetKPsToVisPlat(int channel);
        void SetMPsToVisPlat(int channel, int kfId =-1);
        void SaveVisFile(std::string fileName);
        void SaveVisData(std::string fileName, int channel, int dataType);
        void ClearVisData(int channel, int dataType);
        
        bool checkKF(long frameId);
        //Simple2DVisualFuntion
		void showReprojection(int frameId, const std::string& win_name, bool autoPause);
        void showTrack(long frameId, const std::string& win_name, bool autoPause);
		void showTrack(long frameId, const std::vector<long>& mpIndices, const std::string& win_name, bool autoPause);
        void showTrackAll(long frameId, const std::vector<long>& mpIndices, const std::string& win_name, bool autoPause);
        void showRawKeyPoints(int frameId, const std::string& win_name, bool autoPause);

        // some params currently used for tracking
    public:
        static const int   HISTO_LENGTH = 30;
        const float TH_MAX = 4.0f;  //ssd:(0~4)
        const float TH_ONE_PERCENT = 0.05 * TH_MAX;
        float TH_HIGH;
        float TH_LOW;
        // counter used for represent valid map matches in poseOptimization
        int   numValidMapMatches = 0;

    private:
        // save the mean distance of the KF and preKF (used for cullKeyFrame)
        algo::Mean* kf_center_dis2_;

    protected:
        void  ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

        float fNNRatio;
        bool  mbCheckOrientation;

    public: //todo: make this protected.
//    protected:
        long m_referKFID;
        
    public:
		void searchLocalPoints(long frameId, const std::vector<ygomi::MapPoint*>& plocalMapPoints);
    protected:
//        void searchLocalPoints(long frameId, const std::vector<ygomi::MapPoint*>& plocalMapPoints);
        
        bool isKeyFrameTooClose(long frameId1, long frameId2);
        
        int trackedMapPoints(const int &minObs, Frame* refFrame);
        
        //use 2d-2d matches to fill the globalMap
        void fill2DMatches(long frameId1, long frameId2, const std::vector<std::pair<size_t, size_t> >& indexPairs, std::vector<cv::Point3f>& posiList);
        
        //get 2d match relations between two frame;
        std::vector<std::pair<int, int>> get2DMatches(long reFrameId, long toFrameId);
        
        //delete the connection between mp and kp
        bool DelMpKpConnection(long mpId, long frameId, int kpId);
        bool AddMpKpConnection(long mpId, long frameId, int kpId);
        
        // fuse multi map point indexes(if has) of each key point in certain frame
        void fuseMapPointsKernel(long frameId, int keyFrameCount);

    public:
        void fuseMapPointsKernel(long frameId, const std::vector<long>& mapPointsIndices);

    protected:
        void mergeMapPoints(long frameId, size_t keyId);
        void merge2MapPoints(MapPoint* mpKeep, const MapPoint* mp);
        void merge2Tracks(std::vector<ygomi::Track>& trackFix, const std::vector<ygomi::Track>& trackDel);
        
        void readAllMapPoints(const std::string& file_path, std::vector<ygomi::MapPoint*>& allMapPoints);

		void refineObservations(long frameId);
		
    protected:
        
        void debugTrack(long frameId);
        
        void debugFrame(long frameId);
        
        void analysisSearchByProjection(const ygomi::Frame* frame,
                                          const std::vector<ygomi::MapPoint*> mappoints,
                                          const std::vector<std::pair<size_t, size_t> >& matchingPairs);
		
		void debugMapPoint(long frameId);

        // visualization
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                           float mKeyFrameSize, float mKeyFrameLineWidth, float mGraphLineWidth);
        void DrawMapPoints(float mPointSize, long frameId);

    protected:
        //All map information.
        std::unique_ptr<GlobalMap>               m_pGlobalMap;
        
        //Singleton working instance.
        static std::unique_ptr<ygomi::Platform>  m_pInstance;
        
        //working variables.
        std::unique_ptr<ygomi::FeatureExtractor> m_pFeatureExtractor;
        
        //visualization interface object, using this to save data to visualiztion tool.
        chamo::ViewInterfaceImp* viewerHandler;
        chamo::DataInterface* dataInterface;
        
        //NOTE!!!In general, history frames should not be saved in algorithm, in
        //our case, we need old frames to check history info because of KeyFrames.
        std::unordered_map<long, cv::Mat>        m_frames;
        
        //parse parameters，
        std::unique_ptr<ygomi::ParamParser>      m_pParamParser;

        //camera parameter
        cv::Mat                                   m_K;
        cv::Mat                                   m_K64f;
        cv::Mat                                   m_DistCoef;
        
        //
        static bool                               m_bInit;
		
		// new added map points
        std::vector<long>                    m_newMapPointId;


        //visualization
        bool initState = false;
        pangolin::OpenGlRenderState* ps_cam_;
        pangolin::View* pd_disp_;
    public:
		algo::IGps		   *pIGPS_;
		//std::unique_ptr<algo::IGps> pIGPS_;
		ygomi::outPut* poutput_;
		CommonAlgo::transformTools* pTrsfTool_;
        void showAll(long frameId);
		void getCameraPoses(long begin, long end, std::vector<cv::Mat>& vPose);
		void getCameraCenters(long begin, long end, std::vector<cv::Mat>& vCenter);
		void saveSlamGpsKML(long begin, long end, std::vector<cv::Mat>& vGps,std::vector<cv::Mat>& vCameraPose);
		bool computeTransform(long start, long end);
    };
}
