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
#include "ViewInterfaceImp.h"
#include "paramParser.hpp"
#include "Utils.h"
#include "DataInterface.hpp"

namespace ygomi {
    void Platform::AddActMpToVisPlat(int channel, int mpId, cv::Mat posi, cv::Mat color){
        chamo::ActMPCpp item;
        item.id = mpId;
        item.x = posi.at<float>(0,0);
        item.y = posi.at<float>(0,1);
        item.z = posi.at<float>(0,2);
        item.r = color.at<float>(0,0);
        item.g = color.at<float>(0,1);
        item.b = color.at<float>(0,2);
        viewerHandler->AddActMP(channel, &item);
    }
    
    void Platform::SetKPsToVisPlat(int channel){
        viewerHandler->ClearData(2, channel);
        const std::vector<const ygomi::Frame*> fs =m_pGlobalMap->getAllFrames();
        
        int maxFId=-1;
        for(int j=0;j<fs.size();j++){
            if (maxFId < fs[j]->getId()){
                maxFId = fs[j]->getId();
            }
        }
        ygomi::Frame* pFrame = m_pGlobalMap->getFrame(maxFId);
        if (dataInterface != NULL){
            dataInterface->ClearData();
        }
        while (pFrame){
            if(pFrame->getType() != ygomi::KEY_FRAME){
                pFrame = m_pGlobalMap->findLastFrame(pFrame->getId());
                continue;
            }
            cv::Mat pose = pFrame->getPose();
            if(pose.empty())
            {
                pFrame = m_pGlobalMap->findLastFrame(pFrame->getId());
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
            viewerHandler->AddKeyFrame(channel,&item);
            if (dataInterface != NULL){
                const std::vector<ygomi::KeyPoint>& kps = pFrame->getKeyPoints();
                dataInterface->AddFrame(pFrame->getId(), pFrame->getName(), pFrame->getPose(),pFrame->getType(), kps.size());
                for (int i=0;i<kps.size();i++){
                    std::vector<int> MPIDs;
                    MPIDs.resize(kps[i].m_mapPointId.size());
                    for (int j=0;j<kps[i].m_mapPointId.size();j++){
                        MPIDs[j] = kps[i].m_mapPointId[j];
                    }
                    dataInterface->AddKepPoint(pFrame->getId(), kps[i].m_key.pt,kps[i].m_key.octave ,MPIDs);
                }
            }
            
            pFrame = m_pGlobalMap->findLastFrame(pFrame->getId());
            
        }
    }
    
    void Platform::SetMPsToVisPlat(int channel, int kfId){
        std::vector<ygomi::MapPoint*> mps;
        viewerHandler->ClearData(1, channel);
        if (kfId ==-1){
            mps = m_pGlobalMap->getAllMapPoints();
            
        }else{
            mps = m_pGlobalMap->getMapPointInFrame(kfId);
        }
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
            viewerHandler->AddMapPoint(channel,&item);
            
            if (dataInterface != NULL){
                dataInterface->AddMapPoint(mps[i]->getId(), mps[i]->getPosition(), mps[i]->isBad());
                const std::vector<ygomi::Track>& tracks = mps[i]->getObservation();
                for (int j=0;j<tracks.size();j++){
                    dataInterface->AddTrack(mps[i]->getId(), tracks[j].m_frameId, tracks[j].m_keyPointId);
                }
            }
        }
    }
    
    void Platform::SaveVisData(std::string fileName, int channel, int dataType){
        
        viewerHandler->SaveData(dataType, channel, fileName);
    }
    
    void Platform::ClearVisData(int channel, int dataType){
        viewerHandler->ClearData(dataType, channel);
    }
    
    void Platform::SaveVisFile(std::string fileName){
        if (dataInterface != NULL){
            dataInterface->SaveData(fileName+".xml");
        }
        viewerHandler->SaveAll(fileName);
//       viewerHandler->SaveData(2, 0, fileName);
    }
    
	void Platform::showReprojection(int frameId, const std::string& win_name, bool autoPause){
        Frame* frame = m_pGlobalMap->getFrame(frameId);
		if(!frame)
			return;
        
        const cv::Mat& img = m_frames.find(frameId)->second;
        cv::Mat img1       = img.clone();
        if(img.channels() == 1){
            cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        }else{
            cv::cvtColor(img1, img1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        }
        
        
        const std::vector<ygomi::KeyPoint>& kps = frame->getKeyPoints();
        int kpCount = kps.size();
        int inLierCount = 0;
        int mpCount = 0;
        float err=0;
        for (int i=0;i<kpCount;i++){
            const KeyPoint& kp = frame->getKeyPoint(i);
            for(int k=0; k<kp.m_mapPointId.size(); k++) {
                MapPoint* mp = m_pGlobalMap->getMapPoint(kp.m_mapPointId[k]);
                CV_Assert(mp);
                if (mp->isBad()){
                    continue;
                }
                const cv::Point3f& pos = mp->getPosition();
                const cv::Mat& pose = frame->getPose();
                cv::Mat posiM       = algo::pf3toMat(pos);
                cv::Mat ptHomo      = m_K64f*pose*posiM;
                float repX          = ptHomo.at<double>(0)/ptHomo.at<double>(2);
                float repY          = ptHomo.at<double>(1)/ptHomo.at<double>(2);
                
                cv::Rect ss(cv::Point(repX-5,repY-5), cv::Point(repX+5,repY+5));
                cv::rectangle(img1, ss, cv::Scalar(0,255,0));
                cv::line(img1, cv::Point(kp.m_key.pt.x, kp.m_key.pt.y), cv::Point(repX, repY),cv::Scalar(0,0,255));
                cv::circle(img1,cv::Point(kp.m_key.pt.x, kp.m_key.pt.y),2,cv::Scalar(0,0,255),-1 );
                float delta_x = kp.m_key.pt.x - repX;
                float delta_y = kp.m_key.pt.y - repY;
                err += sqrt(delta_x * delta_x + delta_y * delta_y);
                if(ss.contains(kp.m_key.pt)) {
                    inLierCount++;
                }
                mpCount++;
            }
        }
        err = err/mpCount;
        cv::imshow(win_name, img1);
        std::stringstream idStr;
        idStr<<"FrameId:"<<frameId<<"| MPCount:"<<mpCount<<"| Inlier:"<<inLierCount<<"| AvgErr:"<<err;
        cv::displayStatusBar(win_name, idStr.str());
        if(autoPause){
            cv::waitKey(0);
        }else{
            cv::waitKey(1);
        }
    }
    
	void Platform::showTrack(long frameId, const std::string& win_name, bool autoPause){
		const ygomi::Frame* frame = m_pGlobalMap->getFrame(frameId);
		if(!frame)
			return;
		
        const cv::Mat& gray = m_frames.find(frameId)->second;
        cv::Mat rgbImg = gray.clone();
        if(gray.channels() == 1) {
            cv::cvtColor(gray, rgbImg, cv::COLOR_GRAY2BGR);
        }
        int matches = 0;
        const std::vector<ygomi::KeyPoint>& keys = frame->getKeyPoints();
		for(const auto& key : keys) {
			if(key.m_mapPointId.empty())
				continue;
			for(auto mpId : key.m_mapPointId) {
				MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
                if(mp->isBad())
                    continue;
				std::vector<ygomi::Track> tracks = mp->getObservation();
				auto cmp = [&](const ygomi::Track& t1, const ygomi::Track& t2)->bool{ return t1.m_frameId < t2.m_frameId; };
				std::sort(tracks.begin(), tracks.end(), cmp);
//				int nTrackCount = tracks.size();
//				for(int n=0; n<nTrackCount-1; n++) {
//					//previous frame info
//					const ygomi::Track& prevTrack	= tracks[n];
//					const ygomi::Frame* prevFrame   = m_pGlobalMap->getFrame(prevTrack.m_frameId);
//					CV_Assert(prevFrame);
//					const cv::KeyPoint& prevKey		= prevFrame->getKeyPoint(prevTrack.m_keyPointId).m_key;
//					
//					//current frame info
//					const ygomi::Track& currTrack	= tracks[n+1];
//					const ygomi::Frame* currrFrame	= m_pGlobalMap->getFrame(currTrack.m_frameId);
//					CV_Assert(currrFrame);
//					const cv::KeyPoint& currKey		= currrFrame->getKeyPoint(currTrack.m_keyPointId).m_key;
//					
//					cv::line(rgbImg, prevKey.pt, currKey.pt, cv::Scalar(255, 255, 0));
//					cv::circle(rgbImg, prevKey.pt, 2, cv::Scalar(255, 255, 0), 1);
//				}
            	int n = tracks.size()-1; 
            	if(n>0){
                    matches ++;
            	cv::KeyPoint kfItemPre = m_pGlobalMap->getFrame(tracks[n-1].m_frameId)->getKeyPoint(tracks[n-1].m_keyPointId).m_key;
            	cv::KeyPoint kfItemCurr = m_pGlobalMap->getFrame(tracks[n].m_frameId)->getKeyPoint(tracks[n].m_keyPointId).m_key;
                float dx = std::abs(kfItemPre.pt.x - kfItemCurr.pt.x);
                float dy = std::abs(kfItemPre.pt.y - kfItemCurr.pt.y);
                float distance = std::sqrt(dx*dx + dy *dy);
                if(distance > 100){
                    cv::line(rgbImg, kfItemCurr.pt, kfItemPre.pt, cv::Scalar(0, 0, 255));
//                    std::cout << "curr/pre_location= " << kfItemCurr.pt << " " << kfItemPre.pt << std::endl;
                }else{
                	cv::line(rgbImg, kfItemCurr.pt, kfItemPre.pt, cv::Scalar(255,0 ,0));
                }
                    cv::circle(rgbImg, kfItemPre.pt, 1, cv::Scalar(0, 0, 255), 2);
                    cv::circle(rgbImg, kfItemCurr.pt, 1, cv::Scalar(0, 255, 0), 2);
            	}


			}
//            cv::circle(rgbImg, key.m_key.pt, 1, cv::Scalar(0, 0, 255), 2);
        }
        if (rgbImg.cols > 800)
        {
            float scale = 800.0 / rgbImg.cols;
            cv::resize(rgbImg, rgbImg, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        cv::imshow(win_name, rgbImg);
        long nKFs = m_pGlobalMap->getAllKeyFrames().size();
        std::vector<ygomi::MapPoint*> nMPs;
        nMPs = m_pGlobalMap->getAllMapPoints();
        int valid_MpNum = 0;
        for(size_t i=0; i<nMPs.size(); i++)
        {
			if(!nMPs[i]->isBad())
                valid_MpNum ++;
        }

        std::stringstream idStr;
        idStr <<"SLAM MODE | " << "FrameId: " << frameId << ", KFs: " << nKFs << ", Valid_MPs: " << valid_MpNum
              << ", Matches: " <<  matches;
        cv::displayStatusBar(win_name, idStr.str());
        if(autoPause) {
            cv::waitKey(0);
        }
		else {
            cv::waitKey(1);
        }
    }
	
	void Platform::showTrack(long frameId, const std::vector<long>& mpIndices, const std::string& win_name, bool autoPause)
	{
		Frame* frame = m_pGlobalMap->getFrame(frameId);
		if(!frame)
			return;
		
		const cv::Mat& gray = m_frames.find(frameId)->second;
		cv::Mat rgbImg = gray.clone();
		if(gray.channels() == 1) {
			cv::cvtColor(gray, rgbImg, cv::COLOR_GRAY2BGR);
		}
		
		const std::vector<ygomi::KeyPoint>& kps = frame->getKeyPoints();
		int kpCount = kps.size();
		for(int k=0; k<kpCount;k++) {
			int obCount = kps[k].m_mapPointId.size();
			if(obCount<=0){
				continue;
			}
			for(size_t id=0; id < kps[k].m_mapPointId.size(); id++) {
				long mpId = kps[k].m_mapPointId[id];
				if(std::find(mpIndices.begin(), mpIndices.end(), mpId) == mpIndices.end())
					continue;
				
				MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
				
				std::vector<ygomi::Track> tracks = mp->getObservation();
				auto cmp = [&](const ygomi::Track& t1, const ygomi::Track& t2)->bool{ return t1.m_frameId < t2.m_frameId; };
				std::sort(tracks.begin(), tracks.end(), cmp);
				for(int n=0; n<tracks.size()-1; n++) {
					//previous frame info
					const ygomi::Track& prevTrack	= tracks[n];
					const ygomi::Frame* prevFrame   = m_pGlobalMap->getFrame(prevTrack.m_frameId);
					CV_Assert(prevFrame);
					const cv::KeyPoint& prevKey		= prevFrame->getKeyPoint(prevTrack.m_keyPointId).m_key;
					
					//current frame info
					const ygomi::Track& currTrack	= tracks[n+1];
					const ygomi::Frame* currrFrame	= m_pGlobalMap->getFrame(currTrack.m_frameId);
					CV_Assert(currrFrame);
					const cv::KeyPoint& currKey		= currrFrame->getKeyPoint(currTrack.m_keyPointId).m_key;
					
					cv::line(rgbImg, prevKey.pt, currKey.pt, cv::Scalar(255, 255, 0));
					cv::circle(rgbImg, prevKey.pt, 2, cv::Scalar(255, 255, 0), 1);

				}
				cv::circle(rgbImg, kps[k].m_key.pt, 1, cv::Scalar(0,0,255), 2);
			}
		}

		cv::imshow(win_name, rgbImg);
		std::stringstream idStr;
		idStr << "frameid : " << frameId;
		cv::displayStatusBar(win_name, idStr.str());
		if(autoPause) {
			cv::waitKey(0);
		}
		else {
			cv::waitKey(1);
		}

	}
	
    void Platform::showTrackAll(long frameId, const std::vector<long>& mpIndices, const std::string& win_name, bool autoPause)
    {
        Frame* frame = m_pGlobalMap->getFrame(frameId);
        if(!frame)
            return;
        
        const cv::Mat& img = m_frames.find(frameId)->second;
        cv::Mat img1 = img.clone();
        if(img.channels() == 1) {
            cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        }
        
        const std::vector<ygomi::KeyPoint>& kps = frame->getKeyPoints();
        int kpCount = kps.size();
        int mpCount = 0;
        cv::Scalar colorPoint;
        cv::Scalar colorLine;
        for(int k=0; k<kpCount;k++){
            int obCount = kps[k].m_mapPointId.size();
            if(obCount<=0){
                continue;
            }
            mpCount++;
            for(size_t id=0; id < kps[k].m_mapPointId.size(); id++) {
                long mpId = kps[k].m_mapPointId[id];
                
                if(std::find(mpIndices.begin(), mpIndices.end(), mpId) == mpIndices.end()) {
                    colorPoint = cv::Scalar(0, 0, 255);
                    colorLine = cv::Scalar(255, 255, 0);
                }
                else{
                    colorPoint = cv::Scalar(0, 255, 255);
                    colorLine = cv::Scalar(255, 0, 0);
                }
                
                MapPoint* mp = m_pGlobalMap->getMapPoint(mpId);
                
                std::vector<ygomi::Track> tracks = mp->getObservation();
                auto cmp = [&](const ygomi::Track& t1, const ygomi::Track& t2)->bool{ return t1.m_frameId < t2.m_frameId; };
                std::sort(tracks.begin(), tracks.end(), cmp);
                for(int n=0; n<tracks.size()-1; n++) {
                    cv::KeyPoint kfItem		= m_pGlobalMap->getFrame(tracks[n].m_frameId)->getKeyPoint(tracks[n].m_keyPointId).m_key;
                    cv::KeyPoint kfItemNext = m_pGlobalMap->getFrame(tracks[n+1].m_frameId)->getKeyPoint(tracks[n+1].m_keyPointId).m_key;
                    cv::line(img1, cv::Point(kfItem.pt.x, kfItem.pt.y), cv::Point(kfItemNext.pt.x, kfItemNext.pt.y), colorLine);
                    cv::circle(img1, kfItem.pt, 1, colorLine, 2);
                    cv::circle(img1, kfItemNext.pt, 1, colorLine, 2);
                }
                cv::Point2f p = kps[k].m_key.pt;
                cv::circle(img1, cv::Point(p.x, p.y), 1, colorPoint, 2);
            }
        }
        
        cv::imshow(win_name, img1);
        std::stringstream idStr;
        idStr << "FrameId: " << frameId << " | MPCount: " << mpCount << " | Frame Type: " << frame->getType();
        cv::displayStatusBar(win_name, idStr.str());
        if(autoPause) {
            cv::waitKey(0);
        }
        else {
            cv::waitKey(1);
        }
        
    }
    
	void Platform::showRawKeyPoints(int frameId, const std::string& win_name, bool autoPause) {
        Frame* frame = m_pGlobalMap->getFrame(frameId);
		if(!frame)
			return;
		
        const cv::Mat& img = m_frames.find(frameId)->second;
        cv::Mat img1       = img.clone();
        if(img.channels() == 1)
            cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        
        const std::vector<ygomi::KeyPoint>& kps = frame->getKeyPoints();
        int kpCount = kps.size();
        for(int k=0; k<kpCount;k++){
            cv::Point2f p = kps[k].m_key.pt;
            cv::circle(img1,cv::Point(p.x, p.y),1,cv::Scalar(0,0,255),2);
        }
        cv::imshow(win_name, img1);
        std::stringstream idStr;
        idStr<<"KP Count:"<<kpCount;
        cv::displayStatusBar(win_name,idStr.str());
        if(autoPause){
            cv::waitKey(0);
        }else{
            cv::waitKey(1);
        }
    }
}
