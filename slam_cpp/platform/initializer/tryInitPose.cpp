#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "Platform.hpp"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include "Frame.hpp"
#include "TypeDef.hpp"
#include "optflowIni.h"
#include "Utils.h"
#include "paramParser.hpp"
#include "searchByProjection.hpp"

namespace ygomi {
#define INITE_KP_NUM 100

#ifdef SAVETESTDATA
    const std::string PATHIN = "/Users/test/Testing/MatlabSlam2.0_TestMidianDepth/vehicleSlam/";
    static inline void readNextLine(std::ifstream& fin, std::istringstream& istr)
    {
        std::string line;
        std::getline(fin, line);
        istr.clear();
        istr.str(line);
    }
    
    void readKeyFrame(const std::string& fileName, ygomi::Frame* keyFrame)
    {
        std::string file_path = PATHIN + fileName + ".txt";
        std::ifstream fin;
        fin.open(file_path.c_str());
        if(!fin.is_open())
            throw "cannot open source file.\n";
        
        std::string line, str;
        std::istringstream istr;
        
        //frame id
        readNextLine(fin, istr);
        istr >> str;
        long frameID;
        istr >> str;    frameID = atol(str.c_str());
        
        //refence key frame id.
        readNextLine(fin, istr);
        istr >> str;
        long refID;
        istr >> str;    refID = atol(str.c_str());
        
        //status
        readNextLine(fin, istr);
        istr >> str;
        bool isBad;
        istr >> str;    isBad = atoi(str.c_str());
        ygomi::FrameType type = isBad ? ygomi::BAD_KEY_FRAME : ygomi::KEY_FRAME;
        
        //pose
        cv::Mat pose(3, 4, CV_64FC1);
        std::getline(fin, line);
        for(int j=0; j<3; j++) {
            readNextLine(fin, istr);
            for(int i=0; i<4; i++) {
                istr >> str;
                pose.at<double>(j, i) = atof(str.c_str());
            }
        }
        readNextLine(fin, istr);
        readNextLine(fin, istr); //empty line
        
        readNextLine(fin, istr);
        istr >> str;
        cv::Point3f cameraCenter;
        istr >> str;    cameraCenter.x = atof(str.c_str());
        istr >> str;    cameraCenter.y = atof(str.c_str());
        istr >> str;    cameraCenter.z = atof(str.c_str());
        
        std::vector<ygomi::KeyPoint> keys;
        readNextLine(fin, istr);
        istr >> str;
        int keySize;
        istr >> str;    keySize = atoi(str.c_str());
        for(int i=0; i<keySize; i++) {
            readNextLine(fin, istr);
            cv::KeyPoint key_pt;
            istr >> str;    key_pt.pt.x     = atof(str.c_str());
            istr >> str;    key_pt.pt.y     = atof(str.c_str());
            istr >> str;    key_pt.octave   = atoi(str.c_str());
            istr >> str;    key_pt.class_id = atoi(str.c_str());
            
            std::vector<float> desc_val;
            readNextLine(fin, istr);//descriptor
            while(istr >> str)
                desc_val.push_back(atof(str.c_str()));
            
            cv::Mat desc(1, desc_val.size(), CV_32FC1);
            float* ptr = desc.ptr<float>(0);
            for(auto& val : desc_val)
                *ptr++ = val;
            
            readNextLine(fin, istr);//mp list num
            istr >> str;
            int mpListSize;
            std::vector<long> mpIDLists;
            istr >> str;    mpListSize = atoi(str.c_str());
            for(int i=0; i<mpListSize; i++) {
                readNextLine(fin, istr);
                istr >> str;
                long mpId = atol(str.c_str());
                
                mpIDLists.push_back(mpId);
            }
            
            ygomi::KeyPoint key;
            key.m_key        = key_pt;
            key.m_descriptor = desc;
            key.m_mapPointId = mpIDLists;
            
            keys.push_back(key);
        }
        
        keyFrame->setType(type);
        keyFrame->setPose(pose);
        keyFrame->setRefFrameId(refID);
        keyFrame->addKeys(keys);
    }
    
    void readAllMapPoints(const std::string& fileName, std::vector<ygomi::MapPoint*>& allMapPoints)
    {
        std::string filePath = PATHIN + fileName + ".txt";
        std::ifstream fin;
        fin.open(filePath.c_str());
        if(!fin.is_open())
            throw "cannot open source file";
        
        std::string line, str;
        std::istringstream istr;
        
        readNextLine(fin, istr);
        istr >> str;
        long mpCount;
        istr >> str;    mpCount = atol(str.c_str());
        
        std::vector<long> mpIDList;
        
        for(long id=0; id<mpCount; id++) {
            //status
            readNextLine(fin, istr);
            istr >> str;
            bool isBad;
            istr >> str;    isBad = atoi(str.c_str());
            
            //id
            readNextLine(fin, istr);
            istr >> str;
            long mpId;
            istr >> str;    mpId = atol(str.c_str());
            
            //pt
            readNextLine(fin, istr);
            istr >> str;
            cv::Point3f mp_pt;
            istr >> str;    mp_pt.x = atof(str.c_str());
            istr >> str;    mp_pt.y = atof(str.c_str());
            istr >> str;    mp_pt.z = atof(str.c_str());
            
            //desc
            std::getline(fin, line);
            std::vector<float> desc_val;
            readNextLine(fin, istr);//descriptor.
            while(istr >> str)
                desc_val.push_back(atof(str.c_str()));
            
            cv::Mat desc(1, desc_val.size(), CV_32FC1);
            float* ptr = desc.ptr<float>(0);
            for(auto& val : desc_val)
                *ptr++ = val;
            
            //track size
            std::vector<ygomi::Track> tracks;
            readNextLine(fin, istr);
            istr >> str;
            int trackSize;
            istr >> str;    trackSize = atoi(str.c_str());
            for(int i=0; i<trackSize; i++) {
                readNextLine(fin, istr);
                long frameId;
                istr >> str;    frameId = atol(str.c_str());
                int keyId;
                istr >> str;    keyId = atoi(str.c_str());
                
                ygomi::Track track;
                track.m_frameId    = frameId;
                track.m_keyPointId = keyId;
                
                tracks.push_back(track);
            }
            
            //first_kf_id
            readNextLine(fin, istr);
            istr >> str;
            long firstKFID;
            istr >> str;    firstKFID = atol(str.c_str());
            
            //min_dist
            readNextLine(fin, istr);
            istr >> str;
            float min_distance;
            istr >> str;    min_distance = atof(str.c_str());
            
            //max_dist
            readNextLine(fin, istr);
            istr >> str;
            float max_distance;
            istr >> str;    max_distance = atof(str.c_str());
            
            //norm vector
            readNextLine(fin, istr);
            istr >> str;
            cv::Mat norm(3, 1, CV_32FC1);
            istr >> str;    norm.at<float>(0, 0) = atof(str.c_str());
            istr >> str;    norm.at<float>(1, 0) = atof(str.c_str());
            istr >> str;    norm.at<float>(2, 0) = atof(str.c_str());
            
            //visible
            readNextLine(fin, istr);
            istr >> str;
            int nvisible;
            istr >> str;    nvisible = atoi(str.c_str());
            
            //found
            readNextLine(fin, istr);
            istr >> str;
            int nfound;
            istr >> str;    nfound = atoi(str.c_str());
            
            ygomi::MapPoint* mp = new ygomi::MapPoint();
            mp->setId(mpId);
            mp->setBad(isBad);
            mp->setFirstKeyFrameID(firstKFID);
            mp->addTracks(tracks);
            
            mp->addFound(-mp->getRealFould());
            mp->addFound(nfound);
            
            mp->addVisible(-mp->getVisible());
            mp->addVisible(nvisible);
            
            mp->setPosition(mp_pt);
            mp->setDescriptor(desc);
            
            mp->m_maxDistance = max_distance;
            mp->m_minDistance = min_distance;
            mp->m_normalVector = norm.clone();
            
            allMapPoints.push_back(mp);
            
            CV_Assert(mp->getId() == mpId);
            if(std::find(mpIDList.begin(), mpIDList.end(), mp->getId()) != mpIDList.end())
            {
                int break_point = 0;
            }
            mpIDList.push_back(mp->getId());
        }
        fin.close();
    }
#endif// SAVETESTDATA
    
    bool Platform::tryInitPose(long frameId)
    {
        ygomi::Frame* frame     = m_pGlobalMap->getFrame(frameId);
        static int ntotalFrame  = 0;
        ntotalFrame++;
        
        bool bInit       = false;
        std::vector<ygomi::KeyPoint> vecKPs = frame->getKeyPoints();
        if (vecKPs.size() > INITE_KP_NUM) {
            Frame* lastFrame = frame;
            for (int i=0; i<5; i++){
                lastFrame = m_pGlobalMap->findLastFrame(lastFrame->getId());
                if(!lastFrame) {
                    return false;
                }
                if (lastFrame->getKeyPoints().size() <= INITE_KP_NUM) {
                    continue;
                }
                long lastFrameID = lastFrame->getId();
                if(MatchByOpticFlow(lastFrameID, frame->getId() - lastFrameID)){
#ifdef RUN_TEST_GRID
                    {
                    getGobalMapFromMatlab("/Users/test/Testing/MatlabSlam2.0_TestGrid/vehicleSlam/initialKpMp.mat");
                    computeImageBounds(cv::Rect(0, 0, m_frames[0].cols, m_frames[0].rows), m_K, m_DistCoef);
                    frame = m_pGlobalMap->getFrame(1);
                    assignFeaturesToGrid(frame->getKeyPoints());
                    saveGridInfo("gird");
                    }
#endif //RUN_TEST_GRID
                    bInit = initialMap(lastFrame, frame);
                    if (bInit) {
                        Frame* frameLast = m_pGlobalMap->getFrame(lastFrameID);
                        frameLast->setType(ygomi::KEY_FRAME);
                        m_pGlobalMap->addKeyFrame(frameLast->getId());
                        frame->setType(ygomi::KEY_FRAME);
                        frame->setRefFrameId(frame->getId());
                        m_pGlobalMap->addKeyFrame(frame->getId());
                        m_pGlobalMap->updateCovisibilityGraph(frame->getId());
                        break;
                    }
                }
            }
        }
        //============ test code =============
//        std::cout << "=========After BA==========" << std::endl;
//        optimizePoseByLocalBA(frameId);

        //============ test code =============
        return bInit;
    }
    
    bool Platform::initialMap(Frame* frame1, Frame* frame2)
    {
        CV_Assert(frame1 && frame2 && frame1->getId() < frame2->getId());
        //update mean viewing direction and scale invariance distances.
        const std::vector<ygomi::MapPoint*> mps = m_pGlobalMap->getAllMapPoints();

        for (int i = 0; i < mps.size(); i++) {
            const std::vector<ygomi::Track>& obserations = mps[i]->getObservation();
            if (!obserations.empty()) {
                std::vector<cv::Point3f> cameraCenters;
                std::vector<int> levels;
                const std::vector<float> scaleFactorSet = algo::calcScaleSet(m_pParamParser->parseFloat("raulScaleFactor"), m_pParamParser->parseInteger("raulLevelNum")); //parse input parameters.
                
                int referId = mps[i]->getReferenceKeyFrameID();
                int level = -1;
                for (int i = 0; i < obserations.size(); i++) {
                    ygomi::Frame* mpKF = m_pGlobalMap->getFrame(obserations[i].m_frameId);
                    const ygomi::KeyPoint& mpKP = mpKF->getKeyPoint(obserations[i].m_keyPointId);
                    if (mpKF->getType() == ygomi::KEY_FRAME) {
                        const cv::Point3f cC = mpKF->getCameraCenter();
                        cameraCenters.push_back(cC);
                        if (mpKF->getId() == referId) {
                            level = mpKP.m_key.octave;
                        }
                    }
                }
//                std::cout << "Mp[" << i  << "]";
                mps[i]->updateNormalAndDepth(cameraCenters, level, scaleFactorSet);
                //===================== test log  =====================
//                std::cout << "_maxDist= "<< mps[i]->m_maxDistance ;//<< std::endl;
//                std::cout << "     _minDist= "<< mps[i]->m_minDistance << std::endl;
//                std::cout << " " << mps[i]->m_minDistance << std::endl;
                //===================== test log  =====================
            }
        }
        
        //initial key frame: set median depth to 1(for ORB) or 2(for SGD)
#ifdef RUN_TEST_MEDIANDEPTH
        frame1->resetKeys();
        readKeyFrame("initialFrame", frame1);
        std::vector<ygomi::MapPoint*> allMapPoints;
        readAllMapPoints("allMapPoints", allMapPoints);
        std::vector<cv::Point3f> mpPosiSet;
        for (int i = 0; i < allMapPoints.size(); i++) {
            mpPosiSet.push_back(allMapPoints[i]->getPosition());
        }
#endif// RUN_TEST_MEDIANDEPTH
        std::vector<ygomi::KeyPoint> kps1 = frame1->getKeyPoints();
        std::vector<cv::Point3f> mpPosiSet;
        for (int i = 0; i < kps1.size(); i++) {
            std::vector<long> mpId = kps1[i].m_mapPointId;
            if (mpId.empty()) {
                continue;
            }
            MapPoint* mp = m_pGlobalMap->getMapPoint(mpId[0]);
            mpPosiSet.push_back(mp->getPosition());
        }
        float medianDepth = frame1->computeSceneMedianDepth(mpPosiSet, 2);
        std::cout << "medianDepth: " << medianDepth << std::endl;
        float invMedianDepth;
        switch (ygomi::KeyPoint::m_descType) {
            case ygomi::BINARY_DESC:
                invMedianDepth = 1.0f / medianDepth;
                break;
            case ygomi::FLOAT_DESC:
                invMedianDepth = 2.0f / medianDepth;
                break;
            default:
                invMedianDepth = 2.0f / medianDepth;
                break;
        }
        
        //if need reinit
        if(medianDepth < 0 || frame2->getMapPointIndices().size() < 100)
        {
            m_pGlobalMap->reset();
            std::cout << "Wrong initialization, reseting..." << std::endl;
            return false;
        }
        
        //scale initial baseline
#ifdef RUN_TEST_MEDIANDEPTH
        frame2->resetKeys();
        readKeyFrame("currFrame", frame2);
#endif// RUN_TEST_MEDIANDEPTH
        cv::Mat tPose = frame2->getPose();
//        std::cout << "tPose: " << tPose << "\n";
        tPose.col(3).rowRange(0, 3) = tPose.col(3).rowRange(0, 3) * invMedianDepth; //scale initial baseline
        frame2->setPose(tPose);
//        std::cout << "tPose later: " << tPose << "\n";
        
        //scale points
        for (int i = 0; i < mps.size(); i++) {
            cv::Point3f mpPosi = mps[i]->getPosition();
            mpPosi.x = mpPosi.x * invMedianDepth;
            mpPosi.y = mpPosi.y * invMedianDepth;
            mpPosi.z = mpPosi.z * invMedianDepth;
            mps[i]->setPosition(mpPosi);
        }
        return true;
    }
}
