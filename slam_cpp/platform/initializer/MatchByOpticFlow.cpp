#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "Platform.hpp"
#include "GlobalMap.hpp"
#include "MapPoint.hpp"
#include "Frame.hpp"
#include "TypeDef.hpp"
#include "optflowIni.h"
#include "Utils.h"

namespace ygomi {
#ifdef RUN_TEST_OPTFLOW
    const std::string PATHIN = "/Users/test/Testing/ForShow/TestOpticalFlow/case1/vehicleSlam/";
    const std::string PATHOUT = "/Users/test/Testing/ForShow/TestOpticalFlow/case1/matlabSlam/";
    void saveKP2File(std::string fileName, Frame* mFrame)
    {
        std::string pathName = PATHOUT + fileName + ".txt";
        std::ofstream pf(pathName);
        
        std::string descName = PATHOUT + fileName + "_desc.txt";
        std::ofstream pfDesc(descName);
        
        const std::vector<ygomi::KeyPoint>& kps = mFrame->getKeyPoints();
        for (int i = 0; i < kps.size(); i++) {
            pf << kps[i].m_key.pt.x << " " << kps[i].m_key.pt.y << std::endl;
            cv::Mat desc = kps[i].m_descriptor;
            for (int j = 0; j < desc.cols; j++) {
                pfDesc << desc.at<float>(0, j) << " ";
            }
            pfDesc << "\n";
        }
        pf.close();
        pfDesc.close();
        
        return;
    }
    
    void saveMatches2File(std::string fileName, std::vector<std::pair<std::size_t, std::size_t> > matches)
    {
        std::string pathName = PATHOUT + fileName + ".txt";
        std::ofstream pfOut(pathName);
        
        for (int i = 0; i < matches.size(); i++) {
            pfOut << matches[i].first << " " << matches[i].second << std::endl;
        }
        pfOut.close();
        
        return;
    }
    
    void saveMps2File(std::string fileName, std::vector<cv::Mat> mps)
    {
        std::string pathName = PATHOUT + fileName + ".txt";
        std::ofstream pfOut(pathName);
        
        for (int i = 0; i < mps.size(); i++) {
            pfOut << mps[i].at<float>(0, 0) << " " << mps[i].at<float>(1, 0) << " " << mps[i].at<float>(2, 0) << std::endl;
        }
        pfOut.close();
        
        return;
    }
    
    void savePose(std::string fileName, cv::Mat& R, cv::Mat& t)
    {
        std::string pathName = PATHOUT + fileName + ".txt";
        std::ofstream pfOut(pathName);
        
        for (int i = 0; i < R.rows; i++) {
            for (int j = 0; j < R.cols; j++) {
                pfOut << R.at<float>(i, j) << " ";
            }
            pfOut << "\n";
        }
        pfOut << "\n";
        
        for (int i = 0; i < t.rows; i++) {
            pfOut << t.at<float>(i, 0) << "\n";
        }
        pfOut.close();
        
        return;
    }
    
    void loadKPFromFile(const std::string fileName, const int descriptorDim, const int descriptorType, std::vector<cv::KeyPoint>& kpts,  cv::Mat& descriptor)
    {
        std::string pathName = PATHIN + fileName + ".txt";
        std::ifstream pfIn(pathName);
        std::string descName = PATHIN + fileName + "_desc.txt";
        std::ifstream pfDescIn(descName);
        
        while (pfIn.good()) {
            cv::KeyPoint kp;
            pfIn >> kp.pt.x;
            pfIn >> kp.pt.y;
            kpts.push_back(kp);
        }
        kpts.pop_back();
        pfIn.close();

        descriptor.create(kpts.size(), descriptorDim, descriptorType);
        std::string line;
        for(int row=0; getline(pfDescIn, line); row++) {
            std::istringstream istr(line);
            std::string str;
            float* ptr = descriptor.ptr<float>(row);
            while (istr >> str) {
                *ptr = atof(str.c_str());
                ++ptr;
            }
        }
        return;
    }
#endif // RUN_TEST_OPTFLOW
    
    bool Platform::MatchByOpticFlow(long frameId, int offset)
    {
        long id1 = frameId;
        long id2 = frameId + offset;

        std::vector<cv::KeyPoint> kpts1, kpts2;
        
        Frame* frame1 = m_pGlobalMap->getFrame(id1);
        Frame* frame2 = m_pGlobalMap->getFrame(id2);
        CV_Assert(frame1 && frame2);

        const std::vector<ygomi::KeyPoint>& kps1 = frame1->getKeyPoints();
        const std::vector<ygomi::KeyPoint>& kps2 = frame2->getKeyPoints();
        if(kps1.empty() || kps2.empty())
            return false;
        
        kpts1.reserve(kps1.size());
        
        const int descriptorDim  = kps1[0].m_descriptor.cols;
        const int descriptorType = kps1[0].m_descriptor.type();
        
        cv::Mat desc1(kps1.size(), descriptorDim, descriptorType);
        for(int i=0;i<kps1.size();i++){
            kpts1.push_back(kps1[i].m_key);
            kps1[i].m_descriptor.copyTo(desc1.row(i));
        }
        kpts2.reserve(kps2.size());
        cv::Mat desc2(kps2.size(), descriptorDim, descriptorType);
        for(int i=0;i<kps2.size();i++){
            kpts2.push_back(kps2[i].m_key);
            kps2[i].m_descriptor.copyTo(desc2.row(i));
        }
#ifdef RUN_TEST_OPTFLOW
        kpts1.clear();
        desc1.release();
        loadKPFromFile("initialKeyPoints", descriptorDim, descriptorType, kpts1, desc1);
        kpts2.clear();
        desc2.release();
        loadKPFromFile("currentKeyPoints", descriptorDim, descriptorType, kpts2, desc2);
#endif
        algo::OptflowIni optflowIni_;
        optflowIni_.init(m_K);
        optflowIni_(m_frames[id1], kpts1, desc1);
        optflowIni_(m_frames[id2], kpts2, desc2);
        cv::Mat img1, img2;
        std::vector<std::pair<size_t, size_t> > matches;
        std::vector<cv::Mat> x3Ds;
        cv::Mat R,t;
        
        if(optflowIni_.getIniRet(img1, kpts1, desc1, img2, kpts2, desc2, matches, x3Ds, R, t))
        {
#ifdef RUN_TEST_OPTFLOW
            saveMatches2File("matches", matches);
            saveMps2File("mapPoints", x3Ds);
            savePose("pose", R, t);
            std::cout <<  "opt flow ini success matches num: " << matches.size() << "\n";
            std::cout << "R : " << R << "\n";
            std::cout << "t : " << t << "\n";
#endif // RUN_TEST_OPTICALFLOW
            frame1->setType(KEY_FRAME);
            frame2->setType(KEY_FRAME);
            
            m_newMapPointId.clear();
            for(size_t id = 0; id < matches.size(); id++) {
                const ygomi::KeyPoint& kp1 = frame1->getKeyPoint(matches[id].first);
                //CV_Assert(kp1.m_mapPointId.empty());
                
                const ygomi::KeyPoint& kp2 = frame2->getKeyPoint(matches[id].second);
                //CV_Assert(kp2.m_mapPointId.empty());
                
                //construct a new map points.
                ygomi::MapPoint mp(frame2->getId());
                cv::Point3f pt(x3Ds[id].at<float>(0), x3Ds[id].at<float>(1), x3Ds[id].at<float>(2));
                mp.setPosition(pt);
                mp.setBad(false);
                mp.addTrack(id1, matches[id].first);
                mp.addTrack(id2, matches[id].second);
                
                //merge descriptor
                std::vector<cv::Mat> descriptors;
                descriptors.reserve(2);
                descriptors.push_back(kp1.m_descriptor);
                descriptors.push_back(kp2.m_descriptor);
                mp.mergeDescriptor(descriptors);

                m_pGlobalMap->addMapPoint(mp);
                m_newMapPointId.push_back(mp.getId());
                
                //update keys' connections.
                frame1->addMatchingPair(matches[id].first, mp.getId());
                frame2->addMatchingPair(matches[id].second, mp.getId());
            }

            //set pose to current frame and last frame.
            cv::Mat tPose = algo::makeT(R, t);
            tPose.convertTo(tPose, CV_64FC1);
            tPose= tPose.rowRange(0, 3);
            frame2->setPose(tPose);
            cv::Mat ident=cv::Mat::eye(3, 4, CV_64FC1);
            frame1->setPose(ident);

            return true;
        }
        return false;
    }
}