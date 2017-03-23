#include <stdio.h>
#include <string.h>
#include <vector>
#include "mat.h"
#include "Frame.hpp"
#include "Mappoint.hpp"
#include "GlobalMap.hpp"
#include "ViewInterfaceImp.h"
#include "coreData.hpp"
#include "ReadGlobalMapFromM.hpp"
#include "SaveGlobalMapToM.hpp"
#include "TypeDef.hpp"

namespace ygomi {
    void CoreData::init(){
        m_pGlobalMap = new GlobalMap();
        viewerHandler= new chamo::ViewInterfaceImp();
    }
    
    void CoreData::SaveVisFile(std::string fileAddr){
        std::vector<ygomi::MapPoint*> mps;
        viewerHandler->ClearData(1, 0);
        mps = m_pGlobalMap->getAllMapPoints();
        for (int i=0; i<mps.size(); i++){
            if(mps[i]->isBad()){
                continue;
            }
            cv::Point3f posi = mps[i]->getPosition();
            chamo::MapPointCpp item;
            item.id = mps[i]->getId();
            item.x = posi.x;
            item.y = posi.y;
            item.z = posi.z;
            viewerHandler->AddMapPoint(0,&item);
        }
        
        viewerHandler->ClearData(2, 0);
        
        const std::vector<const ygomi::Frame*> fs =m_pGlobalMap->getAllFrames();
        int maxFId=-1;
        for(int j=0;j<fs.size();j++){
            if (maxFId < fs[j]->getId()){
                maxFId = fs[j]->getId();
            }
        }
        
        ygomi::Frame* pFrame = m_pGlobalMap->getFrame(maxFId);
        while (pFrame){
            if(pFrame->getType() != ygomi::KEY_FRAME){
                continue;
            }
            cv::Mat pose = pFrame->getPose();
            if(pose.empty())
            {
                continue;
            }
            cv::Mat pose44 = cv::Mat::eye(4, 4, CV_64FC1);
            pose.copyTo(pose44.rowRange(0, 3));
            cv::Mat invPose =pose44.inv();
            chamo::KeyFrameCpp item;
            item.id = pFrame->getId();
            item.x = invPose.at<double>(0,3);
            item.y = invPose.at<double>(1,3);
            item.z = invPose.at<double>(2,3);
            item.r00 = invPose.at<double>(0,0);
            item.r01 = invPose.at<double>(0,1);
            item.r02 = invPose.at<double>(0,2);
            item.r10 = invPose.at<double>(1,0);
            item.r11 = invPose.at<double>(1,1);
            item.r12 = invPose.at<double>(1,2);
            item.r20 = invPose.at<double>(2,0);
            item.r21 = invPose.at<double>(2,1);
            item.r22 = invPose.at<double>(2,2);
            viewerHandler->AddKeyFrame(0,&item);
            pFrame = m_pGlobalMap->findLastFrame(pFrame->getId());
        }
        
        viewerHandler->SaveAll(fileAddr);
    }
    
    void CoreData::SaveMat(std::string fileAddr){
        MATFile* pmat = matOpen(fileAddr.c_str(), "w");
        mxArray* pData;
        saveGlobalMapToM(&pData, reinterpret_cast<long>(m_pGlobalMap));
        int status;
        status = matPutVariable(pmat, "coreDataMatrix", pData);
        if (status != 0) {
            printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
            return;
        }
        int fieldCount= mxGetNumberOfFields(pData);
        for (int i=0; i<fieldCount; i++){
            mxArray *subMx = mxGetFieldByNumber(pData, 0, i);
            mxDestroyArray(subMx);
        }
        if (matClose(pmat) != 0) {
            printf("Error closing file %s\n",fileAddr.c_str());
            return;
        }
    }
    
    void CoreData::AddFrame(long frameId, cv::Mat pose, std::string name, bool isKF){
        Frame frame(frameId);
        if (isKF){
            frame.setType(KEY_FRAME);
        }else{
            frame.setType(NORMAL_FRAME);
        }
        
        frame.setPose(pose);
        frame.setName(name);
        m_pGlobalMap->addFrame(frame);
    }
    
    void CoreData::AddKeyPoint(long frameId, cv::KeyPoint kp, cv::Mat desc, std::vector<long> mpList){
        Frame* frame = m_pGlobalMap->getFrame(frameId);
        KeyPoint key;
        key.m_descriptor = desc;
        key.m_mapPointId = mpList;
        key.m_key = kp;
        frame->addKey(key);
    }
    
    void CoreData::AddMappoint(long mpId, cv::Point3f posi, cv::Mat desc, bool isBad){
        MapPoint mapPoint;
        mapPoint.setBad(isBad);
        mapPoint.setId(mpId);
        mapPoint.setPosition(posi);
        mapPoint.setDescriptor(desc);
        m_pGlobalMap->addMapPoint(mapPoint);
    }
    
    void CoreData::AddTrack(long mpId, long frameId, int kpId){
        MapPoint* mapPoint =  m_pGlobalMap->getMapPoint(mpId);
        mapPoint->addTrack(frameId, kpId);
    }
    
}



