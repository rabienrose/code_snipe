/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SaveToGlobalMap.cpp
 * @brief  Save Matlab data structure to C.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.08.04        Zili.Wang     Create
 *******************************************************************************
 */

#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "mat.h"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include "Frame.hpp"
#include "TypeDef.hpp"

#include "SaveGlobalMapToM.hpp"

void saveGlobalMapToM(mxArray** src, long mapHandle)
{
    bool reserveMem = false;
    int maxMPCount = 100000;
    int maxFCount = 200;
    int maxTrackCount = 30;
    int descWidth = 32;
    int descType;
    switch (ygomi::KeyPoint::m_descType) {
        case ygomi::BINARY_DESC:
            descType = CV_8UC1;
            break;
        case ygomi::FLOAT_DESC:
            descType = CV_32FC1;
            break;
        default:
            descType = CV_32FC1;
            break;
    }
    
    ygomi::GlobalMap* pMap = reinterpret_cast<ygomi::GlobalMap*>(mapHandle);
    CV_Assert(pMap);
    const std::vector<const ygomi::Frame*>& frames = pMap->getAllFrames();
    const std::vector<ygomi::MapPoint*>& mappoints = pMap->getAllMapPoints();
    int mpCount = mappoints.size();
    int fCount =frames.size();
    
    int maxMPId = 0;
    for (int i=0;i<mpCount;i++){
        int mpIdC= mappoints[i]->getId();
        if(maxMPId <mpIdC){
            maxMPId = mpIdC;
        }
    }
    
    int maxFId = 0;
    for (int i=0;i<fCount;i++){
        int fIdC= frames[i]->getId();
        if(maxFId <fIdC){
            maxFId = fIdC;
        }
    }
    
    if(!reserveMem){
        maxMPCount = maxMPId+1;
        maxFCount = fCount;
    }
    
    if(fCount>0){
        cv::Mat desc = frames[0]->getKeyPoint(0).m_descriptor;
        descWidth =desc.cols;
        descType =desc.type();
    }
    
    //==================
    //frTypeMatrix
    //==================
    mxArray* frTypeMatrix = mxCreateNumericMatrix(maxFCount, 1, mxDOUBLE_CLASS, mxREAL);
    double* frTypeMatrixBuf = (double* )mxGetData(frTypeMatrix);
    for (int i=0;i<fCount;i++){
        frTypeMatrixBuf[i] = (double)frames[i]->getType();
    }
    
    //==================
    //frImgDirMatrix
    //==================
    mxArray* frImgDirMatrix = mxCreateCellMatrix(maxFCount, 1);
    for (int i=0;i<fCount;i++){
        mxArray* strCell = mxCreateString(frames[i]->getName().c_str());
        mxSetCell(frImgDirMatrix, i, strCell);
    }
    
    //==================
    //frPoseMatrix
    //==================
    mwSize frPoseMatrixSize[3];
    frPoseMatrixSize[0] = 3;
    frPoseMatrixSize[1] = 4;
    frPoseMatrixSize[2] = maxFCount;
    mxArray* frPoseMatrix = mxCreateNumericArray(3, frPoseMatrixSize, mxDOUBLE_CLASS, mxREAL);
    double* frPoseMatrixBuf = (double* )mxGetData(frPoseMatrix);
    
    for (int i=0; i<fCount; i++){
        
        const cv::Mat pose = frames[i]->getPose();
        if(pose.empty()){
            continue;
        }
        if(pose.type() !=CV_64FC1)
        {
            continue;
        }
        cv::Mat poset=pose.t();
        if (poset.isContinuous()){
            std::memcpy(frPoseMatrixBuf+i*12, poset.data , 12*sizeof(double));
        }else{
            std::cout<<"pose matrix not continus!!"<<std::endl;
        }
    }
    
    //==================
    //frImgIdVec
    //==================
    mxArray* frImgIdVec = mxCreateNumericMatrix(maxFId+1, 1, mxDOUBLE_CLASS, mxREAL);
    double* frImgIdVecBuf = (double* )mxGetData(frImgIdVec);
    for (int i=0;i<fCount;i++){
        frImgIdVecBuf[frames[i]->getId()] = i+1;
    }
    
    //==================
    //mpTracksCountVec
    //==================
    mxArray* mpTracksCountVec = mxCreateNumericMatrix(maxMPCount, 1, mxDOUBLE_CLASS, mxREAL);
    double* mpTracksCountVecBuf = (double* )mxGetData(mpTracksCountVec);
    int maxCount = 0;
    for (int i=0;i<mpCount;i++){
        int trackCount = mappoints[i]->getObservation().size();
        if(trackCount > maxCount){
            maxCount = trackCount;
        }
        int mpId= mappoints[i]->getId();
        mpTracksCountVecBuf[mpId] = trackCount;
    }
    if (!reserveMem){
        maxTrackCount = maxCount;
    }
    
    //==================
    //mpIsBadVec
    //==================
    mxArray* mpIsBadVec = mxCreateNumericMatrix(maxMPCount, 1, mxDOUBLE_CLASS, mxREAL);
    double* mpIsBadVecBuf = (double* )mxGetData(mpIsBadVec);
    for (int i=0;i<maxMPCount;i++){
        mpIsBadVecBuf[i] = 1;
    }
    for (int i=0;i<mpCount;i++){
        int mpId= mappoints[i]->getId();
        mpIsBadVecBuf[mpId] = mappoints[i]->isBad();
    }
    
    //==================
    //kpsCountVec
    //==================
    mxArray* kpsCountVec = mxCreateNumericMatrix(maxFCount, 1, mxDOUBLE_CLASS, mxREAL);
    double* kpsCountVecBuf = (double* )mxGetData(kpsCountVec);
    maxCount = 0;
    int maxId = 0;
    for (int i=0;i<fCount;i++){
        const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
        int kpCount = kpList.size();
        if(kpCount > maxCount){
            maxCount = kpCount;
        }
        int id=frames[i]->getId();
        if(id > maxId){
            maxId = id;
        }
        
        kpsCountVecBuf[i] = kpCount;
    }
    int maxKpCount =maxCount;
    
    //==================
    //kpPosiMatrix
    //==================
    mwSize kpPosiMatrixSize[3];
    kpPosiMatrixSize[0] = 2;
    kpPosiMatrixSize[1] = maxKpCount;
    kpPosiMatrixSize[2] = maxFCount;
    mxArray* kpPosiMatrix = mxCreateNumericArray(3, kpPosiMatrixSize, mxDOUBLE_CLASS, mxREAL);
    double* kpPosiMatrixBuf = (double* )mxGetData(kpPosiMatrix);
    for (int i=0; i<fCount; i++){
        const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
        int kpCount = kpList.size();
        for (int j=0; j<kpCount; j++)
        {
            
            kpPosiMatrixBuf[i*kpPosiMatrixSize[0]*kpPosiMatrixSize[1]+j*kpPosiMatrixSize[0] + 0] = (double)kpList[j].m_key.pt.x+1;
            kpPosiMatrixBuf[i*kpPosiMatrixSize[0]*kpPosiMatrixSize[1]+j*kpPosiMatrixSize[0] + 1] = (double)kpList[j].m_key.pt.y+1;
        }
    }
    
    //==================
    //kpDescMatrix
    //==================
//    mwSize kpDescMatrixSize[3];
//    kpDescMatrixSize[0] = descWidth;
//    kpDescMatrixSize[1] = maxKpCount;
//    kpDescMatrixSize[2] = maxFCount;
//    mxArray* kpDescMatrix;
//    if (descType == CV_8UC1){
//        kpDescMatrix = mxCreateNumericArray(3, kpDescMatrixSize, mxUINT8_CLASS, mxREAL);
//        char* kpDescMatrixBuf = (char*)mxGetData(kpDescMatrix);
//        for (int i=0; i<fCount; i++){
//            const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
//            int kpCount = kpList.size();
//            for (int j=0; j<kpCount; j++)
//            {
//                if (kpList[j].m_descriptor.isContinuous()){
//                    
//                    std::memcpy(kpDescMatrixBuf+ i*descWidth*maxKpCount+ j*descWidth, kpList[j].m_descriptor.data , descWidth);
//                }else{
//                    std::cout<<mappoints[i]->getDescriptor()<<std::endl;
//                    std::cout<<"descriptpr not continus!!"<<std::endl;
//                }
//            }
//        }
//    }else if(descType == CV_32F){
//        kpDescMatrix = mxCreateNumericArray(3, kpDescMatrixSize, mxSINGLE_CLASS, mxREAL);
//        float* kpDescMatrixBuf = (float*)mxGetData(kpDescMatrix);
//        for (int i=0; i<fCount; i++){
//            const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
//            int kpCount = kpList.size();
//            for (int j=0; j<kpCount; j++)
//            {
//                if (kpList[j].m_descriptor.isContinuous()){
//                    
//                    std::memcpy(kpDescMatrixBuf+ i*descWidth*maxKpCount+ j*descWidth, kpList[j].m_descriptor.data , descWidth*sizeof(float));
//                }else{
//                    std::cout<<"descriptpr not continus!!"<<std::endl;
//                }
//            }
//        }
//    }
    
    //==================
    //kpOctaveMatrix
    //==================
    mxArray* kpOctaveMatrix = mxCreateNumericMatrix(maxKpCount, maxFCount, mxDOUBLE_CLASS, mxREAL);
    double* kpOctaveMatrixBuf = (double* )mxGetData(kpOctaveMatrix);
    for (int i=0;i<fCount;i++){
        const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
        int kpCount = kpList.size();
        for (int j=0;j<kpCount;j++){
            
            kpOctaveMatrixBuf[i*maxKpCount + j] = kpList[j].m_key.octave;
        }
    }
    
    //==================
    //kpMpIdMatrix
    //==================
    mwSize kpMpIdMatrixSize[3];
    kpMpIdMatrixSize[0] = maxKpCount;
    kpMpIdMatrixSize[1] = 11;
    kpMpIdMatrixSize[2] = maxFCount;
    mxArray* kpMpIdMatrix = mxCreateNumericArray(3, kpMpIdMatrixSize, mxDOUBLE_CLASS, mxREAL);
    double* kpMpIdMatrixBuf = (double* )mxGetData(kpMpIdMatrix);
    for (int i=0; i<fCount; i++){
        const std::vector<ygomi::KeyPoint>& kpList = frames[i]->getKeyPoints();
        for (int j=0;j<kpList.size();j++){
            int tMpCount = kpList[j].m_mapPointId.size();
            kpMpIdMatrixBuf[i*11*maxKpCount+j] = tMpCount;
            for (int k=0;k<tMpCount;k++){
                
                long tmpId = kpList[j].m_mapPointId[k];
                kpMpIdMatrixBuf[i*11*maxKpCount+j+(k+1)*maxKpCount] =tmpId+1;
            }
        }
    }
    
    //==================
    //mpDescMatrix
    //==================
//    mxArray* mpDescMatrix;
//    if (descType == CV_8UC1){
//        mpDescMatrix = mxCreateNumericMatrix(descWidth, maxMPCount, mxUINT8_CLASS, mxREAL);
//        char* mpDescMatrixBuf = (char*)mxGetData(mpDescMatrix);
//        for (int i=0;i<mpCount;i++){
//            if (mappoints[i]->getDescriptor().isContinuous()){
//                int mpId= mappoints[i]->getId();
//                std::memcpy(mpDescMatrixBuf+mpId*descWidth, mappoints[i]->getDescriptor().data , descWidth);
//            }else{
//                std::cout<<mappoints[i]->getDescriptor()<<std::endl;
//                std::cout<<"descriptpr not continus!!"<<std::endl;
//            }
//        }
//    }else if(descType == CV_32F){
//        mpDescMatrix = mxCreateNumericMatrix(descWidth, maxMPCount, mxSINGLE_CLASS, mxREAL);
//        float* mpDescMatrixBuf = (float*)mxGetData(mpDescMatrix);
//        for (int i=0;i<mpCount;i++){
//            if (mappoints[i]->getDescriptor().isContinuous()){
//                int mpId= mappoints[i]->getId();
//                std::memcpy(mpDescMatrixBuf+mpId*descWidth, mappoints[i]->getDescriptor().data , descWidth*sizeof(float));
//            }else{
//                std::cout<<"descriptpr not continus!!"<<std::endl;
//            }
//        }
//    }
    
    //==================
    //mpPosiMatrix
    //==================
    mxArray* mpPosiMatrix = mxCreateNumericMatrix(3, maxMPCount, mxDOUBLE_CLASS, mxREAL);
    double* mpPosiMatrixBuf = (double* )mxGetData(mpPosiMatrix);
    for (int i=0;i<mpCount;i++){
        int mpId= mappoints[i]->getId();
        mpPosiMatrixBuf[mpId*3 + 0] = (double)mappoints[i]->getPosition().x;
        mpPosiMatrixBuf[mpId*3 + 1] = (double)mappoints[i]->getPosition().y;
        mpPosiMatrixBuf[mpId*3 + 2] = (double)mappoints[i]->getPosition().z;
    }
    
    //==================
    //mpTrackMatrix
    //==================
    mwSize mpTrackMatrixSize[3];
    mpTrackMatrixSize[0] = 2;
    mpTrackMatrixSize[1] = maxTrackCount;
    mpTrackMatrixSize[2] = maxMPCount;
    int mpTrackSizeByte = mpTrackMatrixSize[0]*mpTrackMatrixSize[1]*mpTrackMatrixSize[2]*sizeof(int64);
    mxArray* mpTrackMatrix = mxCreateNumericArray(3, mpTrackMatrixSize, mxDOUBLE_CLASS, mxREAL);
    double* mpTrackMatrixBuf = (double* )mxGetData(mpTrackMatrix);
    for (int i=0; i<mpCount; i++){
        int mpId= mappoints[i]->getId();
        const std::vector<ygomi::Track>& track = mappoints[i]->getObservation();
        int trackCount = track.size();
        for (int j=0; j<trackCount; j++)
        {
            mpTrackMatrixBuf[mpId*mpTrackMatrixSize[0]*mpTrackMatrixSize[1]+j*mpTrackMatrixSize[0] + 0] = track[j].m_frameId+1;
            mpTrackMatrixBuf[mpId*mpTrackMatrixSize[0]*mpTrackMatrixSize[1]+j*mpTrackMatrixSize[0] + 1] = (long)track[j].m_keyPointId+1;
        }
    }
    
    mxArray* mpCountM = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double* mpCountBuf = (double* )mxGetData(mpCountM);
    *mpCountBuf = maxMPCount;
    
    mxArray* frCount = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double* frCountBuf = (double* )mxGetData(frCount);
    *frCountBuf = fCount;
    
    int fieldCount = 16;
    char* nameList[fieldCount];
    const char fieldName[][255] = {
        "mpTracksCountVec",
        "mpIsBadVec",
        //"mpDescMatrix",
        "mpPosiMatrix",
        "mpTrackMatrix",
        "frPoseMatrix",
        "frImgIdVec",
        "kpsCountVec",
        "kpPosiMatrix",
        //"kpDescMatrix",
        "kpOctaveMatrix",
        "kpMpIdMatrix",
        "frTypeMatrix",
        "frImgDirMatrix",
        "mpCount",
        "frCount"
    };
    for (int i=0; i<fieldCount; i++){
        nameList[i] =(char*)(fieldName[i]);
    }
    const char** nameListC = (const char**)nameList;
    mxArray * coreDataMatrix = mxCreateStructMatrix(1, 1, fieldCount, nameListC);
    mxSetField(coreDataMatrix, 0, "mpTracksCountVec", mpTracksCountVec);
    mxSetField(coreDataMatrix, 0, "mpIsBadVec", mpIsBadVec);
    //mxSetField(coreDataMatrix, 0, "mpDescMatrix", mpDescMatrix);
    mxSetField(coreDataMatrix, 0, "mpPosiMatrix", mpPosiMatrix);
    mxSetField(coreDataMatrix, 0, "mpTrackMatrix", mpTrackMatrix);
    mxSetField(coreDataMatrix, 0, "frPoseMatrix", frPoseMatrix);
    mxSetField(coreDataMatrix, 0, "frImgIdVec", frImgIdVec);
    mxSetField(coreDataMatrix, 0, "kpsCountVec", kpsCountVec);
    mxSetField(coreDataMatrix, 0, "kpPosiMatrix", kpPosiMatrix);
    //mxSetField(coreDataMatrix, 0, "kpDescMatrix", kpDescMatrix);
    mxSetField(coreDataMatrix, 0, "kpOctaveMatrix", kpOctaveMatrix);
    mxSetField(coreDataMatrix, 0, "kpMpIdMatrix", kpMpIdMatrix);
    mxSetField(coreDataMatrix, 0, "frTypeMatrix", frTypeMatrix);
    mxSetField(coreDataMatrix, 0, "frImgDirMatrix", frImgDirMatrix);
    mxSetField(coreDataMatrix, 0, "mpCount", mpCountM);
    mxSetField(coreDataMatrix, 0, "frCount", frCount);
    
    *src = coreDataMatrix;
}