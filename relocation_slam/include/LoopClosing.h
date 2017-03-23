#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "types_seven_dof_expmap.h"

namespace ORB_SLAM2
{
    class Tracking;
    class LocalMapping;
    class KeyFrameDatabase;
    class LoopClosing
    {
    public:
        
        typedef pair<set<KeyFrame*>,int> ConsistentGroup;
        typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
        
    public:
        LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);
        void SetTracker(Tracking* pTracker);
        void SetLocalMapper(LocalMapping* pLocalMapper);
        void Run();
        void InsertKeyFrame(KeyFrame *pKF);
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);
    protected:
        bool CheckNewKeyFrames();
        bool DetectLoop();
        bool ComputeSim3();
        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);
        void CorrectLoop();
        Map* mpMap;
        Tracking* mpTracker;
        KeyFrameDatabase* mpKeyFrameDB;
        ORBVocabulary* mpORBVocabulary;
        LocalMapping *mpLocalMapper;
        std::list<KeyFrame*> mlpLoopKeyFrameQueue;
        // Loop detector parameters
        float mnCovisibilityConsistencyTh;
        // Loop detector variables
        KeyFrame* mpCurrentKF;
        KeyFrame* mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame*> mvpCurrentConnectedKFs;
        std::vector<MapPoint*> mvpCurrentMatchedPoints;
        std::vector<MapPoint*> mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;
        long unsigned int mLastLoopKFid;
        bool mnFullBAIdx;
    };
    
} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
