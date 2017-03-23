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
 *      2016.08.02        Qiang.Zhai     Create
 *******************************************************************************
 */

#include <mex.h>
//#include <opencvmex.hpp>
#include <opencv2/opencv.hpp>
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "GlobalMap.hpp"
#include "TypeDef.hpp"

///Parse frames.
static void parseFrames(const mxArray* frames, ygomi::GlobalMap*& pMap);

/// Parse map points.
static void parseMapPoints(const mxArray* mapPoints, ygomi::GlobalMap*& pMap);

void readGlobalMapFromM(const mxArray* src, long mapHandle)
{
    CV_Assert(src);
    
    //parse global map handle.
    ygomi::GlobalMap* pMap = reinterpret_cast<ygomi::GlobalMap*>(mapHandle);
    CV_Assert(pMap);
    pMap->reset();
    
    ///parse frames.
    parseFrames(src, pMap);
    
    ///parse map points.
    parseMapPoints(src, pMap);
    
    ///update global map.
    pMap->updateCovisibilityGraph();
    pMap->updateAllKeyFrames();
}

/**
 *@brief In order to unify the ID between C and Matlab, we need to correct used value.
 *       Check the code where function is called.
 */
template<typename T>
static inline T correctValue(T v)
{
    return v - 1;
}

////////////////////////////////////////////////////////////////////////////////
//                  Useful functions for parsing frames.
////////////////////////////////////////////////////////////////////////////////
static void parseFrameIds(const mxArray* frameIdPtr, std::vector<long>& frameId);
static void parseFrameTypes(const mxArray* typesPtr, std::vector<ygomi::FrameType>& frameTypes);
static void parseFrameNames(const mxArray* namesPtr, std::vector<std::string>& frameNames);
static void parseFramePoses(const mxArray* posePtr, std::vector<cv::Mat>& framePoses);
static void parseKeysCountOfFrames(const mxArray* keysCountPtr, std::vector<int>& frameKeysCount);
static void parseKeysPositionOfFrames(const mxArray* keysPosiPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<cv::Point2f> >& frameKeysPosition);
static void parseKeysMPIdOfFrames(const mxArray* keysMpIdPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<std::vector<long> > >& frameKeysMpIds);
static void parseKeysOctavesOfFrames(const mxArray* keysOctavesPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<int> >& frameKeysOctaves);
static void parseKeysDescriptorOfFrames(const mxArray* keysDescriptorPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<cv::Mat> >& frameKeysDesc);

static void parseFrames(const mxArray* frames, ygomi::GlobalMap*& pMap)
{
    mxArray* allFrameIds   = mxGetField(frames, 0, "frImgIdVec"); 	 // maxFrameCount x 1
    std::vector<long> frameIds;
    parseFrameIds(allFrameIds, frameIds);
    
    mxArray* allFrameTypes = mxGetField(frames, 0, "frTypeMatrix"); // frameCount x 1
    std::vector<ygomi::FrameType> frameTypes;
    parseFrameTypes(allFrameTypes, frameTypes);
    
    
    mxArray* allFrameNames = mxGetField(frames, 0, "frImgDirMatrix");  // string x frameCount
    std::vector<std::string> frameNames;
    parseFrameNames(allFrameNames, frameNames);
    
    mxArray* allFramePose  = mxGetField(frames, 0, "frPoseMatrix");  // 3 x 4 x frameCount
    std::vector<cv::Mat> framePoses;
    parseFramePoses(allFramePose, framePoses);
    
    mxArray* allFrameKeyPointsCount = mxGetField(frames, 0, "kpsCountVec"); // keys count of each frame. frameCount x 1
    std::vector<int> keysCountSet;
    parseKeysCountOfFrames(allFrameKeyPointsCount, keysCountSet);
    
    mxArray* allKeysPos    = mxGetField(frames, 0, "kpPosiMatrix"); // 2 x KPCount x frameCount
    std::vector<std::vector<cv::Point2f> > keysPosiSet;
    parseKeysPositionOfFrames(allKeysPos, keysCountSet, keysPosiSet);
    
    mxArray* allKeysMPId   = mxGetField(frames, 0, "kpMpIdMatrix"); // KPCount x (1 + N) x frameCount
    std::vector<std::vector<std::vector<long> > > keysMPIdSet;
    parseKeysMPIdOfFrames(allKeysMPId, keysCountSet, keysMPIdSet);
    
    mxArray* allOctaves    = mxGetField(frames, 0, "kpOctaveMatrix"); //KPCount x frameCount
    std::vector<std::vector<int> > keysOctavesSet;
    parseKeysOctavesOfFrames(allOctaves, keysCountSet, keysOctavesSet);
    
    mxArray* allDescriptor = mxGetField(frames, 0, "kpDescMatrix"); // N x KPCount x frameCount
    std::vector<std::vector<cv::Mat> > keysDescSet;
    parseKeysDescriptorOfFrames(allDescriptor, keysCountSet, keysDescSet);
    
    for(size_t id=0; id<frameIds.size(); id++) {
        int val = frameIds[id];
        if(-1 >= val) continue;
        
        ygomi::Frame frame(id);
        frame.setType(frameTypes[val]);
        frame.setPose(framePoses[val]);
        frame.setName(frameNames[val]);
        
        
        int nCurrKeysCount = keysCountSet[val];
        std::vector<ygomi::KeyPoint> keys;
        keys.reserve(nCurrKeysCount);
        for(size_t j=0; j<nCurrKeysCount; j++) {
            cv::KeyPoint kp;
            kp.pt 	  = keysPosiSet[val][j];
            kp.octave = keysOctavesSet[val][j];
            
            ygomi::KeyPoint key;
            key.m_descriptor = keysDescSet[val][j];
            key.m_mapPointId = keysMPIdSet[val][j];
            key.m_key = kp;
            keys.push_back(key);
        }
        frame.addKeys(keys);
        
        pMap->addFrame(frame);
    }
}

static void parseFrameIds(const mxArray* frameIdPtr, std::vector<long>& frameId)
{
    CV_Assert(frameIdPtr);
    
    size_t frameCount = mxGetNumberOfElements(frameIdPtr);
    frameId.reserve(frameCount);
    double* ptr = static_cast<double*>(mxGetData(frameIdPtr));
    CV_Assert(ptr);
    
    for(size_t i=0; i<frameCount; i++) {
        long id = *ptr++;
        frameId.push_back(id-1);
    }
}

static void parseFrameTypes(const mxArray* typesPtr, std::vector<ygomi::FrameType>& frameTypes)
{
    CV_Assert(typesPtr);
    
    size_t frameCount = mxGetNumberOfElements(typesPtr);
    frameTypes.reserve(frameCount);
    //ygomi::FrameType* ptr = static_cast<ygomi::FrameType*>(mxGetData(typesPtr));
    double* ptr = static_cast<double*>(mxGetData(typesPtr));
    CV_Assert(ptr);
    
    for(size_t i=0; i<frameCount; i++) {
        long dataLong = ptr[i];
        frameTypes.push_back(ygomi::FrameType(dataLong));
    }
}

static void parseFrameNames(const mxArray* namesPtr, std::vector<std::string>& frameNames)
{
    CV_Assert(namesPtr);
    size_t frameCount = mxGetM(namesPtr);
    frameNames.reserve(frameCount);
    
    for(size_t i=0; i<frameCount; i++) {
        const mxArray* strMx = mxGetCell(namesPtr , i);
        int strLenght = mxGetN(strMx);
        char16_t *ss = mxGetChars(strMx);
        std::string tStr;
        for (int n = 0;n<strLenght;n++){
            tStr.push_back(ss[n]);
        }
        frameNames.push_back(tStr);
    }
}

static void parseFramePoses(const mxArray* posePtr, std::vector<cv::Mat>& framePoses)
{
    CV_Assert(posePtr);
    
    size_t elemCount = mxGetNumberOfElements(posePtr);
    const int eachElemSize = 3 * 4;
    size_t frameCount = elemCount / eachElemSize;
    framePoses.reserve(frameCount);
    
    double* ptr = static_cast<double*>(mxGetData(posePtr));
    CV_Assert(ptr);
    for(size_t i=0; i<frameCount; i++) {
        cv::Mat pose(4, 3, CV_64FC1, ptr + i * eachElemSize);
        framePoses.push_back(pose.t());
    }
}

static void parseKeysCountOfFrames(const mxArray* keysCountPtr, std::vector<int>& frameKeysCount)
{
    CV_Assert(keysCountPtr);
    
    size_t frameCount = mxGetNumberOfElements(keysCountPtr);
    frameKeysCount.reserve(frameCount);
    
    double* ptr = static_cast<double*>(mxGetData(keysCountPtr));
    CV_Assert(ptr);
    
    for(size_t i=0; i<frameCount; i++) {
        frameKeysCount.push_back(*ptr++);
    }
}

static void parseKeysPositionOfFrames(const mxArray* keysPosiPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<cv::Point2f> >& frameKeysPosition)
{
    CV_Assert(keysPosiPtr);
    
    const mwSize* pDims = mxGetDimensions(keysPosiPtr);
    mwSize /*s1 = pDims[0],*/ keysCount = pDims[1], frameCount = pDims[2];
    
    CV_Assert(frameCount == frameKeysCount.size()); //check frame count.
    frameKeysPosition.reserve(frameCount);
    
    double* ptr = static_cast<double*>(mxGetData(keysPosiPtr));
    CV_Assert(ptr);
    
    std::vector<cv::Point2f> position;
    for(size_t i=0; i<frameCount; i++) {
        int actualKeysCount = frameKeysCount[i];
        CV_Assert(actualKeysCount <= keysCount);
        
        position.clear();
        position.reserve(actualKeysCount);
        for(size_t j=0; j<actualKeysCount; j++) {
            size_t shift = 2*keysCount*i + 2*j;
            cv::Point2f pt;
            pt.x = *(ptr + shift);
            pt.x = correctValue<float>(pt.x);
            
            pt.y = *(ptr + shift + 1);
            pt.y = correctValue<float>(pt.y);
            
            position.push_back(pt);
        }
        
        frameKeysPosition.push_back(position);
    }
}

static void parseKeysMPIdOfFrames(const mxArray* keysMpIdPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<std::vector<long> > >& frameKeysMpIds)
{
    CV_Assert(keysMpIdPtr);
    const size_t* pDims = mxGetDimensions(keysMpIdPtr);
    size_t keysCount = pDims[0], mapIdsCount = pDims[1], frameCount = pDims[2];
    CV_Assert(frameCount == frameKeysCount.size()); //check frame count.
    frameKeysMpIds.clear();
    frameKeysMpIds.reserve(frameCount);
    
    double* ptr = static_cast<double*>(mxGetData(keysMpIdPtr));
    CV_Assert(ptr);
    
    std::vector<std::vector<long> > keysMpIds;
    for(size_t i=0; i<frameCount; i++) {
        int actualKeysCount = frameKeysCount[i];
        CV_Assert(actualKeysCount <= keysCount);
        
        keysMpIds.clear();
        keysMpIds.reserve(actualKeysCount);
        for(size_t j=0; j<actualKeysCount; j++) {
            
            size_t shift = keysCount * mapIdsCount * i + j;
            
            std::vector<long> mpIds;
            long mpIdCount = *(ptr + shift); // this may be zero.
            
            mpIds.reserve(mpIdCount);
            for(size_t k=0; k<mpIdCount; k++) {
                int mpID = *(ptr + shift + keysCount*(k+1));
                mpID = mpID-1;
                if(mpID<0){
                    std::cout<<"mpID<0!!"<<"mpIdCount:"<<mpIdCount<<" frame:"<<i<<" key:"<<j<<" mpInd:"<<k<<std::endl;
                    continue;
                }
                mpIds.push_back(mpID);
                //std::cout<<mpID<<" ";
            }
            keysMpIds.push_back(mpIds);

        }
        //std::cout<<std::endl;
        frameKeysMpIds.push_back(keysMpIds);
    }
}

static void parseKeysOctavesOfFrames(const mxArray* keysOctavesPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<int> >& frameKeysOctaves)
{
    CV_Assert(keysOctavesPtr);
    const mwSize* pDims = mxGetDimensions(keysOctavesPtr);
    mwSize keysCount = pDims[0], frameCount = pDims[1];
    CV_Assert(frameCount == frameKeysCount.size()); //check frame count.
    
    frameKeysOctaves.reserve(frameCount);
    
    double* ptr = static_cast<double*>(mxGetData(keysOctavesPtr));
    CV_Assert(ptr);
    
    std::vector<int> keysOctaves;
    for(size_t i=0; i<frameCount; i++) {
        int actualKeysCount = frameKeysCount[i]; //actual key points in each frame.
        CV_Assert(actualKeysCount <= keysCount);
        
        keysOctaves.clear();
        keysOctaves.reserve(actualKeysCount);
        for(size_t j=0; j<actualKeysCount; j++) {
            mwSize shift = keysCount * 1 * i + j;
            keysOctaves.push_back(*(ptr + shift));
        }
        
        frameKeysOctaves.push_back(keysOctaves);
    }
}

static void parseKeysDescriptorOfFrames(const mxArray* keysDescriptorPtr, const std::vector<int>& frameKeysCount, std::vector<std::vector<cv::Mat> >& frameKeysDesc)
{
    CV_Assert(keysDescriptorPtr);
    const mwSize* pDims = mxGetDimensions(keysDescriptorPtr);
    mwSize descDim = pDims[0], keysCount = pDims[1], frameCount = pDims[2];
    CV_Assert(frameCount == frameKeysCount.size());
    
    frameKeysDesc.reserve(frameCount);
    
    const char* class_name = mxGetClassName(keysDescriptorPtr);
    int type = -1;
    if(!strcmp("uint8", class_name)) {
        type = CV_8UC1;
    }
    else if(!strcmp("mxDOUBLE_CLASS", class_name)) {char
        type = CV_64FC1;
    }
    else if(!strcmp("single", class_name)){
        type = CV_32FC1;
    }
    CV_Assert(-1 != type);
    
    void* ptr = mxGetData(keysDescriptorPtr);
    
    std::vector<cv::Mat> keysDesc;
    for(size_t i=0; i<frameCount; i++) {
        int actualKeysCount = frameKeysCount[i];
        CV_Assert(actualKeysCount <= keysCount);
        
        keysDesc.clear();
        keysDesc.reserve(actualKeysCount);
        for(size_t j=0; j<actualKeysCount; j++) {
            size_t shift = descDim * keysCount * i + descDim * j;
            cv::Mat desc;
            if(CV_8UC1 == type) {
                desc = cv::Mat(1, static_cast<int>(descDim), type, static_cast<uchar*>(ptr) + shift);
            }
            else if(CV_64FC1 == type) {
                desc = cv::Mat(1, static_cast<int>(descDim), type, static_cast<double*>(ptr) + shift);
            }
            else if(CV_32FC1 == type) {
                desc = cv::Mat(1, static_cast<int>(descDim), type, static_cast<float*>(ptr) + shift);
            }
            keysDesc.push_back(desc.clone());
        }
        
        frameKeysDesc.push_back(keysDesc);
    }
}

////////////////////////////////////////////////////////////////////////////////
//                  Useful functions for parsing map points.
////////////////////////////////////////////////////////////////////////////////
static void parseMPCount(const mxArray* mpCountPtr, long& count);
static void parseMPPosition(const mxArray* mpPosiPtr, std::vector<cv::Point3f>& mpPosition);
static void parseMPStatus(const mxArray* mpStatusPtr, std::vector<bool>& mpIsBad);
static void parseMPDescriptor(const mxArray* mpDescPtr, std::vector<cv::Mat>& mpDesc);
static void parseMPTracksCount(const mxArray* mpTracksCountPtr, std::vector<int>& mpTracksCount);
static void parseMPTracks(const mxArray* mpTracksPtr, const std::vector<int>& mpTracksCount, std::vector<std::vector<ygomi::Track> >& mpTracks);

static void parseMapPoints(const mxArray* mapPoints, ygomi::GlobalMap*& pMap)
{
    mxArray* mpCountPtr 	  = mxGetField(mapPoints, 0, "mpCount");
    long mpCount;
    parseMPCount(mpCountPtr, mpCount);
    
    mxArray* mpPosiPtr 		  = mxGetField(mapPoints, 0, "mpPosiMatrix");
    std::vector<cv::Point3f> mpPosition;
    parseMPPosition(mpPosiPtr, mpPosition);
    
    mxArray* mpStatusPtr 	  = mxGetField(mapPoints, 0, "mpIsBadVec");
    std::vector<bool> mpIsBad;
    parseMPStatus(mpStatusPtr, mpIsBad);
    
    mxArray* mpDescriptorPtr  = mxGetField(mapPoints, 0, "mpDescMatrix");
    std::vector<cv::Mat> mpDesc;
    parseMPDescriptor(mpDescriptorPtr, mpDesc);
    
    mxArray* mpTracksCountPtr = mxGetField(mapPoints, 0, "mpTracksCountVec");
    std::vector<int> mpTracksCount;
    parseMPTracksCount(mpTracksCountPtr, mpTracksCount);
    
    mxArray* mpTracksPtr      = mxGetField(mapPoints, 0, "mpTrackMatrix");
    std::vector<std::vector<ygomi::Track> > mpTracks;
    parseMPTracks(mpTracksPtr, mpTracksCount, mpTracks);
    
    for(size_t i=0; i<mpCount; i++) {
        ygomi::MapPoint mapPoint(0); //todo: 
        if (!mpDesc[i].empty()){
            mapPoint.setDescriptor(mpDesc[i]);
        }
        mapPoint.setBad(mpIsBad[i]);
        mapPoint.setId(i);
        mapPoint.setPosition(mpPosition[i]);
        
        for(size_t j=0; j<mpTracksCount[i]; j++) {
            mapPoint.addTrack(mpTracks[i][j]);
        }
        pMap->addMapPoint(mapPoint);
    }
}

static void parseMPCount(const mxArray* mpCountPtr, long& count)
{
    CV_Assert(mpCountPtr);
    double* ptr = static_cast<double*>(mxGetData(mpCountPtr));
    count = *ptr;
}

static void parseMPPosition(const mxArray* mpPosiPtr, std::vector<cv::Point3f>& mpPosition)
{
    CV_Assert(mpPosiPtr);
    
    cv::Mat position;
    
    double* ptr = static_cast<double*>(mxGetData(mpPosiPtr));
    int countM = mxGetM(mpPosiPtr);
    int countN = mxGetN(mpPosiPtr);
    
    cv::Point3f pt;
    for(int i=0; i<countN; i++) {
        pt.x = *(ptr+i*3+0);
        pt.y = *(ptr+i*3+1);
        pt.z = *(ptr+i*3+2);
        
        mpPosition.push_back(pt);
    }
}

static void parseMPStatus(const mxArray* mpStatusPtr, std::vector<bool>& mpIsBad)
{
    CV_Assert(mpStatusPtr);
    
    cv::Mat status;
    //ocvMxArrayToImage_bool(mpStatusPtr, status);
    double* ptr = static_cast<double*>(mxGetData(mpStatusPtr));
    int count = mxGetNumberOfElements(mpStatusPtr);
    for(int i=0; i<count; i++) {
        mpIsBad.push_back(ptr[i]);
    }
}

static void parseMPDescriptor(const mxArray* mpDescPtr, std::vector<cv::Mat>& mpDesc)
{
    CV_Assert(mpDescPtr);
    
    const char* class_name = mxGetClassName(mpDescPtr);
    if(!strcmp("single", class_name)) {
        float* ptr = static_cast<float*>(mxGetData(mpDescPtr));
        int countM = mxGetM(mpDescPtr);
        int countN = mxGetN(mpDescPtr);
        for(int i=0; i<countN; i++) {
            cv::Mat desc(1, countM, CV_32FC1);
            std::memcpy(desc.data, ptr+i*countM, countM*sizeof(float));
            mpDesc.push_back(desc);
        }
    }
    else if(!strcmp("double", class_name)){
        double* ptr = static_cast<double*>(mxGetData(mpDescPtr));
        int countM = mxGetM(mpDescPtr);
        int countN = mxGetN(mpDescPtr);
        for(int i=0; i<countN; i++) {
            cv::Mat desc(1, countM, CV_64FC1);
            std::memcpy(desc.data, ptr+i*countM, countM*sizeof(double));
            mpDesc.push_back(desc);
        }
    }
    else if(!strcmp("uint8", class_name)) {
        unsigned char* ptr = static_cast<unsigned char*>(mxGetData(mpDescPtr));
        int countM = mxGetM(mpDescPtr);
        int countN = mxGetN(mpDescPtr);
        for(int i=0; i<countN; i++) {
            cv::Mat desc(1, countM, CV_8UC1);
            std::memcpy(desc.data, ptr+i*countM, countM);
            mpDesc.push_back(desc);
        }
    }
}

static void parseMPTracksCount(const mxArray* mpTracksCountPtr, std::vector<int>& mpTracksCount)
{
    CV_Assert(mpTracksCountPtr);
    
    double* ptr = static_cast<double*>(mxGetData(mpTracksCountPtr));
    int count = mxGetNumberOfElements(mpTracksCountPtr);
    for(int i=0; i<count; i++) {
        mpTracksCount.push_back(ptr[i]);
    }
}

static void parseMPTracks(const mxArray* mpTracksPtr, const std::vector<int>& mpTracksCount, std::vector<std::vector<ygomi::Track> >& mpTracks)
{
    CV_Assert(mpTracksPtr);
    const mwSize* pDims = mxGetDimensions(mpTracksPtr);
    mwSize /*descDim = pDims[0],*/ tracksCount = pDims[1], mpCount = pDims[2];
    CV_Assert(mpCount == mpTracksCount.size());
    
    double* ptr = static_cast<double*>(mxGetData(mpTracksPtr));
    CV_Assert(ptr);
    
    std::vector<ygomi::Track> tracks;
    for(size_t i=0; i<mpCount; i++) {
        int actualTrackCount = mpTracksCount[i];
        CV_Assert(actualTrackCount <= tracksCount);
        
        tracks.clear();
        tracks.reserve(actualTrackCount);
        ygomi::Track track;
        for(size_t j=0; j<actualTrackCount; j++) {
            mwSize shift = 2 * tracksCount * i + 2 * j;
            track.m_frameId		= *(ptr + shift);
            track.m_frameId     = track.m_frameId-1;

            track.m_keyPointId  = *(ptr + shift + 1);
            track.m_keyPointId  = track.m_keyPointId-1;
            
            tracks.push_back(track);
        }
        
        mpTracks.push_back(tracks);
    }
}
