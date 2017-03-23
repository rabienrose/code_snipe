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

namespace ygomi {
    void Platform::fill2DMatches(long frameId1, long frameId2, const std::vector<std::pair<size_t, size_t> > &indexPairs, std::vector<cv::Point3f>& posiList)
    {
        int i=0;
        for(const auto& pair : indexPairs) {
            ygomi::Frame* frame1 = m_pGlobalMap->getFrame(frameId1);
            CV_Assert(frame1);
            const ygomi::KeyPoint& kp1 = frame1->getKeyPoint(pair.first);
            //CV_Assert(kp1.m_mapPointId.empty());
            
            ygomi::Frame* frame2 = m_pGlobalMap->getFrame(frameId2);
            CV_Assert(frame2);
            const ygomi::KeyPoint& kp2 = frame2->getKeyPoint(pair.second);
            //CV_Assert(kp2.m_mapPointId.empty());
            
            //construct a new map points.
            ygomi::MapPoint mp(frameId2);
            mp.setBad();
            mp.addTrack(frameId1, pair.first);
            mp.addTrack(frameId2, pair.second);
            
            //merge descriptor of map point.
            std::vector<cv::Mat> descs;
            descs.reserve(2);
            descs.push_back(frame1->getKeyPoint(pair.first).m_descriptor.clone());
            descs.push_back(frame2->getKeyPoint(pair.second).m_descriptor.clone());
            mp.mergeDescriptor(descs);
            if (i<posiList.size()){
                mp.setBad(false);
                mp.setPosition(posiList[i]);
            }
            
            m_pGlobalMap->addMapPoint(mp);
            m_newMapPointId.push_back(mp.getId());
            
            //update keys' connections.
            frame1->addMatchingPair(pair.first, mp.getId());
            frame2->addMatchingPair(pair.second, mp.getId());
            i++;
        }
    }
}



