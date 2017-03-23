#include "CoreData.h"
#include <tuple>
#include "algo_bf.hpp"

namespace chamo
{
    void CoreData::clearAll(){
        data.kfList.clear();
        data.mpList.clear();
    }
    
    int CoreData::save(std::string fileAddress){
        algo::FBufferBin bf(fileAddress, algo::FBufferBin::write);
        if(!bf.good()){
            return -1;
        }
        
        bf.w() & data;
        return 1;
    }
    
    int CoreData::read(std::string fileAddress){
        algo::FBufferBin bf(fileAddress, algo::FBufferBin::read);
        if(!bf.good())
            return -1;
        bf.r() & data;
        return 1;
    }
    
    cv::KeyPoint* CoreData::getKP(int frameId, int kpId){
        return &data.kfList[frameId].kpList[kpId].kp;
    }
    
    cv::KeyPoint* CoreData::getKPByTrack(TrackItemChamo pair){
        return getKP(pair.frameId, pair.kpOrderId);
    }
    
    cv::KeyPoint* CoreData::getKPByTrack(int trackId, int orderInTrack){
        TrackItemChamo pair = data.mpList[trackId].track[orderInTrack];
        return getKPByTrack(pair);
    }
    
    std::vector<TrackItemChamo>& CoreData::getTrack(int trackId){
        return data.mpList[trackId].track;
    }
    
    int CoreData::getMPCount(){
        return data.mpList.size();
    }
    
    MapPointChamo* CoreData::getMPObj(int mpId){
        return &data.mpList[mpId];
    }
    
    int CoreData::getFrameCount(){
        return data.kfList.size();
    }
    
    KeyFrameChamo* CoreData::getKFObj(int frameId){
        return &data.kfList[frameId];
    }
    
    int CoreData::getKPCount(int frameId){
        if(frameId>=data.kfList.size()){
            return 0;
        }
        return data.kfList[frameId].kpList.size();
    }
    
    void CoreData::clearMPList(){
        data.mpList.clear();
    }
    
    void CoreData::addTrack(std::vector<TrackItemChamo>& track){
        MapPointChamo item;
        item.track =track;
        item.isBad = false;
        data.mpList.push_back(item);
    }
    
    void CoreData::setMPSize(int count){
        data.mpList.reserve(count);
    }
    
    void CoreData::addKF(KeyFrameChamo& kf){
        data.kfList.push_back(kf);
    }
    
    void CoreData::setKFSize(int count){
        data.kfList.resize(count);
    }
    
    void CoreData::addKP(KeyPointChamo& kp, int frameId){
        if(frameId >= data.kfList.size()){
            std::cout<<"addKP no frameId!"<<std::endl;
            return;
        }
        data.kfList[frameId].kpList.push_back(kp);
    }
    
    KeyPointChamo* CoreData::getKPObj(int frameId, int kpId){
        if(frameId >= data.kfList.size()){
            std::cout<<"getKPObj no frameId!"<<std::endl;
            return NULL;
        }
        return &data.kfList[frameId].kpList[kpId];
    }
}//
