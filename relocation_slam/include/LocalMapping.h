#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
namespace ORB_SLAM2
{
    
    class Tracking;
    class LoopClosing;
    class Map;
    class LocalMapping
    {
    public:
        LocalMapping(Map* pMap);
        void SetLoopCloser(LoopClosing* pLoopCloser);
        void SetTracker(Tracking* pTracker);
        void InsertKeyFrame(KeyFrame* pKF);
    protected:
        void Run();
        void ProcessNewKeyFrame();
        void CreateNewMapPoints();
        void MapPointCulling();
        void SearchInNeighbors();
        void KeyFrameCulling();
        cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
        Map* mpMap;
        LoopClosing* mpLoopCloser;
        Tracking* mpTracker;
        std::list<KeyFrame*> mlNewKeyFrames;
        KeyFrame* mpCurrentKeyFrame;
        std::list<MapPoint*> mlpRecentAddedMapPoints;
    };
} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
