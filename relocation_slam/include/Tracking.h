#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "System.h"

namespace ORB_SLAM2
{
    
    class Map;
    class LocalMapping;
    class LoopClosing;
    class System;
    
    class Tracking
    {
        
    public:
        Tracking(System* pSys, ORBVocabulary* pVoc,Map* pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath);
        cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
        void SetLocalMapper(LocalMapping* pLocalMapper);
        void SetLoopClosing(LoopClosing* pLoopClosing);
    public:
        
        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3
        };
        
        eTrackingState mState;
        
        // Current Frame
        Frame mCurrentFrame;
        cv::Mat mImGray;
        
        // Initialization Variables (Monocular)
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;
        
        Map* mpMap;
        cv::Mat mK;
        cv::Mat mDistCoef;
        float mbf;
        
    protected:
        void Track();
        void MonocularInitialization();
        void CreateInitialMapMonocular();
        bool TrackReferenceKeyFrame();
        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();
        
        bool TrackLocalMap();
        void SearchLocalPoints();
        
        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();
        
        //Other Thread Pointers
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopClosing;
        
        //ORB
        ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
        ORBextractor* mpIniORBextractor;
        
        //BoW
        ORBVocabulary* mpORBVocabulary;
        KeyFrameDatabase* mpKeyFrameDB;
        
        // Initalization (only for monocular)
        Initializer* mpInitializer;
        
        //Local Map
        KeyFrame* mpReferenceKF;
        std::vector<KeyFrame*> mvpLocalKeyFrames;
        std::vector<MapPoint*> mvpLocalMapPoints;
        
        // System
        System* mpSystem;
        
        
        //Current matches in frame
        int mnMatchesInliers;
        
        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame* mpLastKeyFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        
        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;
        
        list<MapPoint*> mlpTemporalPoints;
    };
    
} //namespace ORB_SLAM

#endif // TRACKING_H
