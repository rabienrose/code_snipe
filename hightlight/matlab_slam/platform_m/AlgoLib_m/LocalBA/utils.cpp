//
//  utils.cpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/21/16.
//  Copyright © 2016 zhaiq. All rights reserved.
//

#include "utils.hpp"
#include <fstream>

void save(const char* file, const std::unordered_map<long, ygomi::Frame>& frames)
{
    ///Open output stream.
    std::ofstream fout;
    fout.open(file);
    if(!fout.is_open()) {
        std::cout << "Cannot open the output stream\n";
    }
    
    ///Save frames
    for(auto it : frames) {
        fout << "===================Frame start===================\n";
        
        fout << "FrameID: " << it.first << "\n";
        fout << "Camera Pose: \n";
        for(int i=0; i<3; i++) {
            for(int j=0; j<4; j++) {
                fout << it.second.pose.at<float>(i, j) << " ";
            }
            fout << "\n";
        }
        
        fout << "KeysCount: " << it.second.keyPoints.size() << "\n";
        for(int i=0; i<it.second.keyPoints.size(); i++) {
            fout << "mapPointID: " << it.second.keyPoints[i].mapPointId << "    ";
            const cv::KeyPoint& key = it.second.keyPoints[i].pt;
            fout << "keyInfo: " << key.pt.x << " " << key.pt.y << " " << key.octave << "\n";
        }
        
        fout << "===================Frame end===================\n\n\n";
    }
    
    ///Close file stream.
    fout.close();
}

void save(const char* file, const std::unordered_map<long, ygomi::MapPoint>& mappoints)
{
    ///Open output stream.
    std::ofstream fout;
    fout.open(file);
    if(!fout.is_open()) {
        std::cout << "Cannot open the output stream\n";
    }
    
    ///Save map points.
    for(auto it : mappoints) {
        fout << "****************map point start****************\n";
        fout << "mapPointID: " << it.first << "\n";
        fout << "3dPoint: " << it.second.pt.x << " " << it.second.pt.y << " " << it.second.pt.z << "\n";
        
        fout << "TrackCount: " << it.second.tracks.size() << "\n";
        for(auto track : it.second.tracks) {
            fout << "frameID: " << track.frameId << " keyID: " << track.keyPointId << "\n";
        }
        fout << "\n";
    }
    
    ///Close file stream.
    fout.close();
}