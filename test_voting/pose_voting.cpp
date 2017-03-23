#include "pose_voting.h"
#include <iostream>
#include <cmath>
#include<opencv2/opencv.hpp>

#define N 200
#define N_step 10
float round_c(float input){
    return round(input/N_step)*N_step;
}

cell_t getCell(cv::Point2f queryPt, cv::Point2f centerPt){
    cv::Point2f relPos =queryPt -centerPt;
    return cell_t(round_c(relPos.x), round_c(relPos.y));
}

cv::Point2f getCellCenter(cell_t cell, cv::Point2f centerPt){
    return centerPt+cv::Point2f(cell.x,cell.y);
}

bool checkInShape(cv::Point2f queryPt, cv::Point3f posi, float minD, float maxD){
    cv::Point2f centerPt(posi.x, posi.y);
    float dis =sqrt((queryPt.x-posi.x)*(queryPt.x-posi.x)+(queryPt.y-posi.z)*(queryPt.y-posi.z));
    if (minD<dis && dis<maxD){
        return true;
    }else{
        return false;
    }
}

void VotingSpace::getDepthRange(cv::Mat K, cv::Point3f posi, cv::Point2f uv, float pitchAng, float eer_h, float cam_h, float eer_r, float &minD, float &maxD){
    cv::Mat rotM= cv::Mat::eye(3,3,CV_32FC1);
    rotM.at<float>(1,1)=std::cos(pitchAng);
    rotM.at<float>(2,2)=std::cos(pitchAng);
    rotM.at<float>(1,2)=-std::sin(pitchAng);
    rotM.at<float>(2,1)=std::sin(pitchAng);
    //std::cout<<rotM<<std::endl;
    cv::Mat H =K*rotM*K.inv();
    //std::cout<<H<<std::endl;
    cv::Mat ptM = cv::Mat(3,1,CV_32FC1);
    ptM.at<float>(0,0) =uv.x;
    ptM.at<float>(1,0) =uv.y;
    ptM.at<float>(2,0) =1;
    cv::Mat alignPt = H*ptM;
    cv::Point2f alignUV(alignPt.at<float>(0,0), alignPt.at<float>(1,0));
    //std::cout<<uv.y-alignUV.y<<":"<<uv.y<<std::endl;
    float cy=K.at<float>(1,2);
    float fy=K.at<float>(1,1);
    float y=posi.y;
    cv::Mat diss(1,4,CV_32FC1);
    diss.at<float>(0,0) = fy*(y-eer_h-cam_h)/((alignUV.y+eer_r)-cy);
    diss.at<float>(0,1) = fy*(y-eer_h-cam_h)/((alignUV.y-eer_r)-cy);
    diss.at<float>(0,2) = fy*(y+eer_h-cam_h)/((alignUV.y+eer_r)-cy);
    diss.at<float>(0,3) = fy*(y+eer_h-cam_h)/((alignUV.y-eer_r)-cy);
    double minDD, maxDD;
    cv::minMaxIdx(diss, &minDD, &maxDD);
    minD =minDD;
    maxD =maxDD;
}

void VotingSpace::addErrShape(cv::Point3f posi, float minD, float maxD, int mpid){
    float dis =sqrt((gps.x-posi.x)*(gps.x-posi.x)+(gps.y-posi.y)*(gps.y-posi.y));
    for (int u=-N; u<=N; u=u+N_step){
        for (int v=-N; v<=N; v=v+N_step){
            cell_t cell(u,v);
            cv::Point2f queryPt=getCellCenter(cell, gps);
            bool inShape = checkInShape(queryPt, posi, minD, maxD);
            if(inShape){
                if (votingSpace.count(cell)==0){
                    std::vector<int> idList;
                    idList.push_back(mpid);
                    votingSpace.insert({cell,idList});
                }else{
                    votingSpace[cell].push_back(mpid);
                }
            }
        }
    }
}

bool VotingSpace::checkRange(cv::Mat pose){

}

cv::Mat VotingSpace::visVotingSpace(std::set<int>& candi_pairs){
    int maxVoteCount = 0;
    typedef std::unordered_map<cell_t, std::vector<int>>::iterator it_type;
    cell_t maxCell;
    std::vector<int> maxInliers;
    for(it_type iterator = votingSpace.begin(); iterator != votingSpace.end(); iterator++) {
        if (iterator->second.size()>maxVoteCount){
            maxVoteCount =iterator->second.size();
            maxCell =iterator->first;
            maxInliers =iterator->second;
        }
    }
    int topThres= maxVoteCount*0.9;
    std::vector<cell_t> tops;
    std::vector<cell_t> maxs;
    for(it_type iterator = votingSpace.begin(); iterator != votingSpace.end(); iterator++) {
        if(iterator->second.size() == maxVoteCount){
            maxs.push_back(iterator->first);
            for (int i=0;i<iterator->second.size();i++){
                candi_pairs.insert(iterator->second[i]);
            }
            continue;
        }
        if(iterator->second.size() >= topThres){
            tops.push_back(iterator->first);
        }
    }
    
    std::cout<<"maxVoteCount: "<<maxVoteCount<<std::endl;
    cv::Mat img = cv::Mat::zeros(N/N_step*2+1, N/N_step*2+1, CV_8UC1);
    for (int u=-N; u<=N; u=u+N_step){
        for (int v=-N; v<=N; v=v+N_step){
            if (votingSpace.count(cell_t(u,v))!=0){
                img.at<unsigned char>((u+N)/N_step,(v+N)/N_step) = votingSpace[cell_t(u,v)].size()/(float)maxVoteCount*255;
            }
        }
    }
    cv::cvtColor(img, img, CV_GRAY2BGRA);
    for(int i=0;i<tops.size();i++){
        cv::circle(img, cv::Point((tops[i].y+N)/N_step, (tops[i].x+N)/N_step), 0, cv::Scalar(0, 255, 0, 255), 1);
    }
    for(int i=0;i<maxs.size();i++){
        cv::circle(img, cv::Point((maxs[i].y+N)/N_step, (maxs[i].x+N)/N_step), 0, cv::Scalar(0, 0, 255, 255), 1);
    }
    //cv::resize(img, img, cv::Size(img.rows*2, img.cols*2));
    //cv::namedWindow("Pose Voting");
    //cv::setWindowProperty("Pose Voting", CV_WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);
    std::stringstream ss;
    ss<<"Max Voted Count: "<<maxVoteCount;
    //cv::displayStatusBar("Pose Voting", ss.str());
    return img;
}
