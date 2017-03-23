//
//  LocalBA.cpp
//  Matlab SLAM
//
//  Created by zhaiq on 7/18/16.
//  Copyright (@) 2016 ygomi. All rights reserved.
//

#include "LocalBA_m.hpp"
#include <opencvmex.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "LocalBAHandle.hpp"
#include "utils.hpp"

//#define SAVE_INPUT

static bool parseKeyFrames(const mxArray* keyFrameInfo, std::unordered_map<long, ygomi::Frame>& keyFrames);
static bool parseMapPoints(const mxArray* mapPointsInfo, std::unordered_map<long, ygomi::MapPoint>& mapPoints);
static void writeBack(const std::unordered_map<long, ygomi::Frame>& keyFrames, const std::unordered_map<long, ygomi::MapPoint>& maapPoints, mxArray* plhs[]);

void mexFunction(int nlhs, mxArray* plhs[],
				 const int nrhs, const mxArray* prhs[])
{
    try {
        if(nrhs < 4 || nlhs < 2) {
            ERROR_LOG("not enough input in LocalBA, please check!\n");
            DEFAULT_OUTPUT(nlhs, plhs);
            return;
        }
        
        //parse key frames.
        std::unordered_map<long, ygomi::Frame> keyFrames;
        if(!parseKeyFrames(prhs[0], keyFrames)) {
            ERROR_LOG("invalid key frames input in LocalBA!\n");
            DEFAULT_OUTPUT(nlhs, plhs);
            return;
        }
        
        //parse map points.
        std::unordered_map<long, ygomi::MapPoint> mapPoints;
        if(!parseMapPoints(prhs[1], mapPoints)) {
            ERROR_LOG("invalid map points input in LocalBA!\n");
            DEFAULT_OUTPUT(nlhs, plhs);
            return;
        }
        
        //parse camera parameters.
        cv::Mat K;
        ocvMxArrayToMat_double(prhs[2], K);
        K.convertTo(K, CV_32FC1);
                
        //parse scale.
        int N = mxGetNumberOfElements(prhs[3]);
        double* ptr = static_cast<double*>(mxGetData(prhs[3]));
        
        std::vector<float> scaleFactor;
        scaleFactor.reserve(N);
        
        std::vector<float> invScaleFactorSigma2;
        invScaleFactorSigma2.reserve(N);
        
        for(int i=0; i<N; i++) {
            scaleFactor.push_back(static_cast<float>(ptr[i]));
            invScaleFactorSigma2.push_back(1.0f / (scaleFactor[i] * scaleFactor[i]));
        }
        
        // save("mex_input_key_frame.txt", keyFrames);
        // save("mex_input_map_point.txt", mapPoints);

        localBAHandle(keyFrames, mapPoints, K, invScaleFactorSigma2);
        
        // save("mex_output_key_frame.txt", keyFrames);
        // save("mex_output_map_point.txt", mapPoints);
        
        writeBack(keyFrames, mapPoints, plhs);
    } catch (...) {
        ERROR_LOG("Caught a general exception in LocalBA!");
    }
}

const char* keyFramesFieldNames[]   = {"frameId",   "pose",         "keys"};
const char* keyPointFieldNames[]    = {"mpId",      "pt",     "octave"};

static bool parseKeyFrames(const mxArray* keyFrameInfo, std::unordered_map<long, ygomi::Frame>& keyFrames)
{
#ifdef SAVE_INPUT
    std::ofstream fout;
    const char* file_name = "input_key_frame.txt";
    fout.open(file_name);
#endif

    if(!keyFrameInfo) return false;
    
    ygomi::Frame frame;
    keyFrames.clear();
    mwSize frameCount = mxGetNumberOfElements(keyFrameInfo);
    for(size_t id=0; id<frameCount; id++) {
        //read frame id.
        mxArray* frameIdPtr = mxGetField(keyFrameInfo, id, keyFramesFieldNames[0]);
        if(!frameIdPtr || mxGetClassID(frameIdPtr) != mxDOUBLE_CLASS || mxIsComplex(frameIdPtr))
            return false;
        long frameId = static_cast<long>(mxGetScalar(frameIdPtr));
        
#ifdef SAVE_INPUT        
        fout << "===================Frame start===================\n";
        fout << "FrameID: " << frameId << std::endl;
#endif

        //read pose.
        mxArray* pose3DPtr  = mxGetField(keyFrameInfo, id, keyFramesFieldNames[1]);
        if(!pose3DPtr) return false;
        ocvMxArrayToMat_double(pose3DPtr, frame.pose);
        frame.pose.convertTo(frame.pose, CV_32FC1);
        
#ifdef SAVE_INPUT
        fout << "Camera Pose: \n";
        for(int j=0; j<3; j++) {
            for(int i=0; i<4; i++) {
                fout << frame.pose.at<float>(j, i) << " ";
            }
            fout << "\n";
        }
#endif

        //read keys of frame.
        frame.keyPoints.clear();
        
        mxArray* keysPtr    = mxGetField(keyFrameInfo, id, keyFramesFieldNames[2]);
        if(!keysPtr) return false;
        mwSize keysCount = mxGetNumberOfElements(keysPtr);
        
#ifdef SAVE_INPUT
        fout << "KeysCount: " << keysCount << std::endl;
#endif

        for(mwSize keyId=0; keyId < keysCount; keyId++) {
            //
            mxArray* mpIdPtr    = mxGetField(keysPtr, keyId, keyPointFieldNames[0]);
            if(!mpIdPtr || mxGetClassID(mpIdPtr) != mxDOUBLE_CLASS || mxIsComplex(mpIdPtr))
                return false;
            long mapPointId = static_cast<long>(mxGetScalar(mpIdPtr));

#ifdef SAVE_INPUT            
            fout << "mapPointID: " << mapPointId << "    ";
#endif            
            //
            cv::KeyPoint key;
            mxArray* ptPtr      = mxGetField(keysPtr, keyId, keyPointFieldNames[1]);
            if(!ptPtr)  return false;
            const float* val    = static_cast<float*>(mxGetData(ptPtr));
            if(!val)    return false;
            key.pt.x = static_cast<float>(val[0]) - 1;
            key.pt.y = static_cast<float>(val[1]) - 1;
            
            //
            mxArray* octavePtr  = mxGetField(keysPtr, keyId, keyPointFieldNames[2]);
            if(!octavePtr)  return false;
            key.octave          = static_cast<int>(mxGetScalar(octavePtr));
            
#ifdef SAVE_INPUT
            fout << "keyInfo: " << key.pt.x << " " << key.pt.y << " " << key.octave << "\n";
#endif            
            ygomi::KeyPoint ygo_key;
            ygo_key.mapPointId = mapPointId;
            //ygo_key.trackId = trackId;
            ygo_key.pt = key;
            
            frame.keyPoints.push_back(ygo_key);
        }
#ifdef SAVE_INPUT        
        fout << "===================Frame end===================\n\n\n";
#endif        
        keyFrames[frameId] = frame;
    }
    
#ifdef SAVE_INPUT
    fout.close();
#endif

    return true;
}

const char* mapPointsFieldNames[]   = {"id",        "pt",      "tracks"};
const char* trackFieldNames[]       = {"frameId",   "keyId"};

static bool parseMapPoints(const mxArray* mapPointsInfo, std::unordered_map<long, ygomi::MapPoint>& mapPoints)
{
    if(!mapPointsInfo) return false;

#ifdef SAVE_INPUT
    std::ofstream fout;
    const char* file_name = "input_map_point.txt";
    fout.open(file_name);
#endif

    mwSize mapPointsCount = mxGetNumberOfElements(mapPointsInfo);
    
    mapPoints.clear();
    ygomi::MapPoint mapPoint;
    
    for(int id=0; id<mapPointsCount; id++) {
        //
        mxArray* idPtr = mxGetField(mapPointsInfo, id, mapPointsFieldNames[0]);
        if(!idPtr || mxGetClassID(idPtr) != mxDOUBLE_CLASS || mxIsComplex(idPtr))
            return false;
        long mapPointId = static_cast<long>(mxGetScalar(idPtr));

#ifdef SAVE_INPUT        
        fout << "****************map point start****************\n";
        fout << "mapPointID: " << mapPointId << std::endl;
#endif        
        //
        mxArray* ptPtr = mxGetField(mapPointsInfo, id, mapPointsFieldNames[1]);
        if(!ptPtr) return false;
        cv::Mat cvPt;
        ocvMxArrayToMat_single(ptPtr, cvPt);
        
        mapPoint.pt.x = cvPt.at<float>(0, 0);
        mapPoint.pt.y = cvPt.at<float>(0, 1);
        mapPoint.pt.z = cvPt.at<float>(0, 2);
        
#ifdef SAVE_INPUT
        fout << "3dPoint: " << mapPoint.pt.x << "  " << mapPoint.pt.y << "  " << mapPoint.pt.z << "\n";
#endif        
        //
        mxArray* trackPtr = mxGetField(mapPointsInfo, id, mapPointsFieldNames[2]);
        if(!trackPtr) return false;
        mwSize trackCount = mxGetNumberOfElements(trackPtr);
        
#ifdef SAVE_INPUT        
        fout << "TrackCount: " << trackCount << "\n";
#endif

        mapPoint.tracks.clear();
        for(mwSize i=0; i<trackCount; i++) {
            //
            mxArray* frameIdPtr = mxGetField(trackPtr, i, trackFieldNames[0]);
            if(!frameIdPtr)
                return false;
            long frameId = static_cast<long>(mxGetScalar(frameIdPtr));
#ifdef SAVE_INPUT            
            fout << "frameID: " << frameId << "    ";
#endif            
            //
            mxArray* keyIdPtr   = mxGetField(trackPtr, i, trackFieldNames[1]);
            if(!keyIdPtr)
                return false;
            int keyId = static_cast<int>(mxGetScalar(keyIdPtr)) - 1;
#ifdef SAVE_INPUT            
            fout << "keyID: " << keyId << "\n";
#endif
            //
            ygomi::Track track;
            track.frameId       = frameId;
            track.keyPointId    = keyId;
            
            mapPoint.tracks.push_back(track);
            
        }
#ifdef SAVE_INPUT        
        fout << "****************map point end****************\n\n";
#endif        
        mapPoint.isBad = false;        
        mapPoints[mapPointId] = mapPoint;
    }
#ifdef SAVE_INPUT
    fout.close();
#endif    
    //save("in_parseMapPoints.txt", mapPoints);
    
    return true;
}

static void writeBack(const std::unordered_map<long, ygomi::Frame>& keyFrames, const std::unordered_map<long, ygomi::MapPoint>& mapPoints, mxArray* plhs[])
{
    //write key frames back.
    const int numOfKeyFrameFields = 3;
    mwSize dims[2] = {static_cast<mwSize>(keyFrames.size()), 1};
    plhs[0] = mxCreateStructArray(2, dims, numOfKeyFrameFields, keyFramesFieldNames);
    int idFieldIdx   = mxGetFieldNumber(plhs[0], keyFramesFieldNames[0]);
    int poseFieldIdx = mxGetFieldNumber(plhs[0], keyFramesFieldNames[1]);
    int keysFieldIdx = mxGetFieldNumber(plhs[0], keyFramesFieldNames[2]);
    int loop = 0;
    for(auto it : keyFrames) {
        mxArray* idPtr = mxCreateDoubleScalar(static_cast<double>(it.first)); //notice, frame id.
        mxSetFieldByNumber(plhs[0], loop, idFieldIdx, idPtr);
        
        cv::Mat pose;
        it.second.pose.convertTo(pose, CV_64FC1);
        mxArray* posePtr = ocvMxArrayFromMat_double(pose);
        mxSetFieldByNumber(plhs[0], loop, poseFieldIdx, posePtr);
        
        //update keys
        const int numOfKeysFields = 3;
        mwSize keysDims[2] = {static_cast<mwSize>(it.second.keyPoints.size()), 1};
        mxArray* keysPtr = mxCreateStructArray(2, keysDims, numOfKeysFields, keyPointFieldNames);

        //const char* keyPointFieldNames[]    = {"mpId",      "pt",     "octave"};
        int mpIdIdx     = mxGetFieldNumber(keysPtr, keyPointFieldNames[0]);
        int ptIdx       = mxGetFieldNumber(keysPtr, keyPointFieldNames[1]);
        int octaveIdx   = mxGetFieldNumber(keysPtr, keyPointFieldNames[2]);
        int key_loop    = 0;
        for(auto iter : it.second.keyPoints) {
            int mapPointId = iter.mapPointId;
            mxArray* mpIdPtr = mxCreateDoubleScalar(static_cast<double>(mapPointId)); //notice
            mxSetFieldByNumber(keysPtr, key_loop, mpIdIdx, mpIdPtr);

            cv::Mat pt(1, 2, CV_32FC1);
            pt.at<float>(0, 0) = iter.pt.pt.x + 1;     pt.at<float>(0, 1) = iter.pt.pt.y + 1;
            mxArray* ptPtr = ocvMxArrayFromMat_single(pt);
            mxSetFieldByNumber(keysPtr, key_loop, ptIdx, ptPtr);

            mxArray* octavePtr = mxCreateDoubleScalar(static_cast<double>(iter.pt.octave));
            mxSetFieldByNumber(keysPtr, key_loop, octaveIdx, octavePtr);

            key_loop++;
        }
        mxSetFieldByNumber(plhs[0], loop, keysFieldIdx, keysPtr);

        loop++;
    }
    
    //write map points back.
    const int numOfMapPointFields = 3;
    dims[0] = static_cast<mwSize>(mapPoints.size());
    plhs[1] = mxCreateStructArray(2, dims, numOfMapPointFields, mapPointsFieldNames);
    int mapPointIdField  = mxGetFieldNumber(plhs[1], mapPointsFieldNames[0]);
    int positionFieldIdx = mxGetFieldNumber(plhs[1], mapPointsFieldNames[1]);
    int tracksFieldIdx   = mxGetFieldNumber(plhs[1], mapPointsFieldNames[2]);
    loop = 0;
    for(auto it : mapPoints) {
        mxArray* idPtr = mxCreateDoubleScalar(static_cast<double>(it.first)); //notice, map point id.
        mxSetFieldByNumber(plhs[1], loop, mapPointIdField, idPtr);
        
        cv::Mat position(3, 1, CV_32FC1);
        position.at<float>(0, 0) = it.second.pt.x;
        position.at<float>(1, 0) = it.second.pt.y;
        position.at<float>(2, 0) = it.second.pt.z;
        mxArray* positionPtr = ocvMxArrayFromMat_single(position);
        mxSetFieldByNumber(plhs[1], loop, positionFieldIdx, positionPtr);
        
        //update tracks of map points.
        const int numOfTracksFields = 2;
        mwSize trackDim[2] = {static_cast<mwSize>(it.second.tracks.size()), 1};
        mxArray* tracksPtr = mxCreateStructArray(2, trackDim, numOfTracksFields, trackFieldNames);

        int frameIdIdx  = mxGetFieldNumber(tracksPtr, trackFieldNames[0]);
        int keyIdIdx    = mxGetFieldNumber(tracksPtr, trackFieldNames[1]);
        int track_loop = 0;
        for(auto iter : it.second.tracks) {
            mxArray* frameIdPtr = mxCreateDoubleScalar(static_cast<double>(iter.frameId));
            mxSetFieldByNumber(tracksPtr, track_loop, frameIdIdx, frameIdPtr);

            mxArray* keyIdPtr   = mxCreateDoubleScalar(static_cast<double>(iter.keyPointId + 1));
            mxSetFieldByNumber(tracksPtr, track_loop, keyIdIdx, keyIdPtr);

            track_loop++;
        }
        mxSetFieldByNumber(plhs[1], loop, tracksFieldIdx, tracksPtr);

        loop++;
    }
}