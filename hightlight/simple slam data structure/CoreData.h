#pragma once
#include <vector>
#include <string>
#include "opencv2/core.hpp"

namespace chamo
{
    struct TrackItemChamo{
        int frameId;
        int kpOrderId;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &frameId &kpOrderId;
        }
    };
    
    struct KeyPointChamo{
        cv::KeyPoint kp;
        int orderInTrack;
        int TrackId;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &kp &orderInTrack &TrackId;
        }
    };
    
    struct MapPointChamo{
        cv::Mat posi;
        std::vector<TrackItemChamo> track;
        bool isBad;
        bool hasPosi(){return !isBad && !posi.empty();};
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &posi &track &isBad;
        }
    };
    
    struct KeyFrameChamo{
        cv::Mat poseLocal;
        cv::Mat poseWorld;
        cv::Mat groundTurth;
        cv::Mat gpsPosi;
        std::vector<KeyPointChamo> kpList;
        bool isBad;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &poseLocal &poseWorld &groundTurth &gpsPosi &kpList &isBad;
        }
    };
    
    struct FeatureMatchData{
        std::vector<KeyFrameChamo> kfList;
        std::vector<MapPointChamo> mpList;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &kfList &mpList;
        }
    };
    
    class CoreData{
    public:
        void clearAll();
        int save(std::string fileAddress);
        int read(std::string fileAddress);
        //kp related
        KeyPointChamo* getKPObj(int frameId, int kpId);
        int getKPCount(int frameId);
        cv::KeyPoint* getKP(int frameId, int kpId);
        cv::KeyPoint* getKPByTrack(int trackId, int orderInTrack);
        cv::KeyPoint* getKPByTrack(TrackItemChamo pair);
        void addKP(KeyPointChamo& kp, int frameId);
        
        //kf related
        KeyFrameChamo* getKFObj(int frameId);
        int getFrameCount();
        void setKFSize(int count);
        void addKF(KeyFrameChamo& kf);
        
        //mp related++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++6
        MapPointChamo* getMPObj(int mpId);
        std::vector<TrackItemChamo>& getTrack(int mpId);
        int getMPCount();
        void setMPSize(int count);
        void addTrack(std::vector<TrackItemChamo>& track);
        void clearMPList();
    private:
        FeatureMatchData data;
    };
    

}//
