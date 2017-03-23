#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2
{
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;
    
    class System
    {
    public:
        
    public:
        System(const string &strVocFile, const string &strSettingsFile);
        cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);
        int GetTrackingState();
        std::vector<MapPoint*> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
        void VisMatch2d(cv::Mat img1, cv::Mat img2, std::string strSettingPath);
        Map* mpMap;
        Tracking* mpTracker;
    private:
        ORBVocabulary* mpVocabulary;
        KeyFrameDatabase* mpKeyFrameDatabase;
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopCloser;
        int mTrackingState;
        std::vector<MapPoint*> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    };
    
}

#endif // SYSTEM_H
