#include "DataInterfaceImp.hpp"
#include "XmlSLAM.hpp"
#include <string>
#include <iostream>

namespace chamo
{
    DataInterfaceImp::DataInterfaceImp(){
        xmlObj = new XmlSLAM();
    }
    void DataInterfaceImp::AddFrame(int id, std::string name, cv::Mat pose, int type, int kpCount){
        if(pose.type()!= CV_32FC1){
            pose.convertTo(pose, CV_32FC1);
        }
        Frame frame;
        frame.name = name;
        frame.id = id;
        frame.pose = pose;
        frame.type = type;
        frame.kps.reserve(kpCount);
        xmlObj->frames[id] = frame;
    }
    void DataInterfaceImp::AddKepPoint(int frameId, cv::Point2f uv, int octave, std::vector<int> MPIDs){
//        if(xmlObj->frames.count(frameId)==0){
//            return;
//        }
//        KeyPoint kp;
//        kp.MPIDs = MPIDs;
//        kp.uv = uv;
//        kp.octave = octave;
//        xmlObj->frames[frameId].kps.push_back(kp);
    }
    void DataInterfaceImp::AddMapPoint(int id, cv::Point3f posi, int isBad){
//        MapPoint mp;
//        mp.id = id;
//        mp.posi = posi;
//        mp.isBad = isBad;
//        xmlObj->mapoints[id] = mp;
    }
    void DataInterfaceImp::AddTrack(int mpId, int frameId, int kpId){
//        if(xmlObj->mapoints.count(mpId)==0){
//            return;
//        }
//        xmlObj->mapoints[mpId].tracks.push_back(std::pair<int, int>(frameId, kpId));
    }
    
    void DataInterfaceImp::ClearData(){
        xmlObj->ClearData();
    }
    void DataInterfaceImp::SaveData(std::string fileName){
        xmlObj->SaveData(fileName);
    }
}
