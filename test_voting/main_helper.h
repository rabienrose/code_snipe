#ifndef MAIN_HELPER
#define MAIN_HELPER
#include <iostream>
#include <string>
#include "AlgoInterfaceImp.h"
#include <opencv2/opencv.hpp>
#include "DbgViewer.h"
#include "DbgMapDrawer.h"
#include "ORBextractor.h"
#include "Config.h"
#include "SGDmatcher.h"
#include "OrbMap.h"
#include "visualisation/ViewInterfaceImp.h"
#include "se3.hpp"
#include <ctime>
#include <set>

class LocData: public algo::SlamData{
public:
    std::vector<int> frameIds;
    std::vector<std::vector<int>> kpList;
};

std::vector<double> unique(const cv::Mat& input, bool sort = true)
{
    if (input.channels() > 1 || input.type() != CV_64F)
    {
        std::cerr << "unique !!! Only works with CV_64F 1-channel Mat" << std::endl;
        return std::vector<double>();
    }
    
    std::vector<double> out;
    for (int y = 0; y < input.rows; ++y)
    {
        const double* row_ptr = input.ptr<double>(y);
        for (int x = 0; x < input.cols; ++x)
        {
            float value = row_ptr[x];
            
            if ( std::find(out.begin(), out.end(), value) == out.end() )
                out.push_back(value);
        }
    }
    
    if (sort)
        std::sort(out.begin(), out.end());
    
    return out;
}

cv::Mat readImg(std::string traj_name, int indx){
    std::string imageRoot = "/Volumes/chamo/dataset/gm_new/images";
    std::stringstream ss;
    ss<<imageRoot+"/"+traj_name+"/"<< std::setw (4)<< std::setfill ('0')<<indx<<".jpg";
    cv::Mat img1;
    cv::imread(ss.str(), cv::IMREAD_GRAYSCALE).copyTo(img1);
    cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
    return img1;
}

cv::Mat plotKP(cv::Mat img, LocData &data, int frameId){
    
}

void getKF2FMapping(algo::AlgoRTMatrixIn *fs, std::vector<int>& mapList){
    int startId = fs->getStartFrameIndex();
    for (int i=startId; i<1800;i++){
        bool isKf;
        cv::Mat poseTrue;
        fs->get(i, poseTrue, isKf);
        if(isKf){
            mapList.push_back(i);
        }
    }
}

bool getEstPosi(cv::Point2f gps, std::vector<cv::Point2f>& kfs1, cv::Point2f& re, int& fid){
    float min_dis = 99999;
    for(int frameId=0; frameId<kfs1.size();frameId=frameId+1){
        cv::Point2f disVec =kfs1[frameId]-gps;
        float dis =sqrt(disVec.x*disVec.x+disVec.y*disVec.y);
        if (dis<min_dis){
            min_dis = dis;
            fid = frameId;
        }
    }
    if (min_dis<15){
        re = kfs1[fid];
        return true;
    }
    return false;
}

void genLocKF(algo::AlgoRTMatrixIn &rts, algo::AlgoKeyFrameIn &kfs, LocData &locData){
    kfs.getKFs(locData.vKFD_);
    kfs.getMPs(locData.vMP3D_);
}
typedef Eigen::Matrix<double,6,1> Vector6d;
cv::Mat getCenter(cv::Mat se3s)
{
    cv::Mat center(se3s.cols,1,CV_64FC1);
    for (int i = 0; i<se3s.cols; i++){
        center.at<double>(i,0) = cv::mean(se3s.col(i))[0];
    }
    return center;
}

Vector6d getLie(cv::Mat pose_M){
    cv::Mat pose;
    pose_M.convertTo(pose, CV_64FC1);
    Eigen::Matrix<double,3,3> ro_eigen;
    ro_eigen(0,0) = pose.at<double>(0,0);
    ro_eigen(0,1) = pose.at<double>(0,1);
    ro_eigen(0,2) = pose.at<double>(0,2);
    ro_eigen(1,0) = pose.at<double>(1,0);
    ro_eigen(1,1) = pose.at<double>(1,1);
    ro_eigen(1,2) = pose.at<double>(1,2);
    ro_eigen(2,0) = pose.at<double>(2,0);
    ro_eigen(2,1) = pose.at<double>(2,1);
    ro_eigen(2,2) = pose.at<double>(2,2);
    Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
    Sophus::SE3d cSE3(ro_eigen, t);
    Vector6d se3=cSE3.log();
    return se3;
}

cv::Mat avi_pose(std::vector<cv::Mat> poses){
    cv::Mat se3s(poses.size(), 6, CV_64FC1);
    for(int j=0;j<poses.size();j++){
        Vector6d se3 = getLie(poses[j]);
        for (int i=0;i<6;i++){
            se3s.at<double>(j,i) =se3(i,0);
        }
    }
    cv::Mat center = getCenter(se3s);
    Vector6d center6d;
    for (int i=0;i<6;i++){
        center6d(i,0) =center.at<double>(i,0);
    }
    Sophus::SE3d centerSE3 = Sophus::SE3d::exp(center6d);
    
    
    Sophus::SE3d::Transformation poseTran = centerSE3.matrix();
    cv::Mat centerM(4,4,CV_32FC1);
    for (int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            centerM.at<float>(i,j)=poseTran(i,j);
        }
    }
    return centerM;
}

int doPnp(std::vector<cv::Point3f> &mps, std::vector<cv::Point2f> &kps, cv::Mat &K, cv::Mat &DistCoef, cv::Mat &re, std::set<int>& inliers, cv::Point2f estPosi){
    int pairCount=5000;
    if (mps.size()<20){
        return -1;
    }
    std::set<unsigned long> check_repeat;
    cv::Mat img = cv::Mat::zeros(400, 400, CV_8UC1);
    //cv::cvtColor(img, img, CV_GRAY2BGRA);
    std::vector<cv::Point2f> all_pose;
    std::vector<std::vector<int>> groups;
    std::vector<cv::Mat> all_pose_M;
    std::vector<Vector6d> all_pose_Lie;
    std::vector<int> pair_used_count;
    pair_used_count.resize(mps.size());
    for(int j=0;j<pairCount;j++){
        cv::Mat rvec;
        cv::Mat tvec;
        std::vector<cv::Point3f> objectPoints;
        objectPoints.resize(4);
        std::vector<cv::Point2f> imagePoints;
        imagePoints.resize(4);
        unsigned long choose[4];
        bool check_rand_suc=true;
        for (int i=0;i<4;i++){
            choose[i] = round(std::rand()/(float)RAND_MAX*(mps.size()-1));
            for (int n=0;n<i;n++){
                if(n!=i){
                    if(choose[n]==choose[i]){
                        check_rand_suc=false;
                        break;
                    }
                }
            }
        }
        if (check_rand_suc==false){
            continue;
        }
        unsigned long token = 0;
        token = token + choose[0];
        token = token + choose[1]*1000;
        token = token + choose[2]*1000000;
        token = token + choose[3]*1000000000;
        if (check_repeat.find(token)==check_repeat.end()){
            check_repeat.insert(token);
        }else{
            continue;
        }
        for (int i=0;i<4;i++){
            objectPoints[i] = mps[choose[i]];
            imagePoints[i] = kps[choose[i]];
            pair_used_count[choose[i]] = pair_used_count[choose[i]]+1;
        }
        cv::solvePnP(objectPoints, imagePoints, K, DistCoef, rvec, tvec, false, CV_P3P);
        cv::Mat rotM;
        cv::Rodrigues(rvec, rotM);
        cv::Mat pose = cv::Mat::eye(4, 4, CV_64FC1);
        rotM.copyTo(pose.colRange(0, 3).rowRange(0, 3));
        tvec.copyTo(pose.col(3).rowRange(0, 3));
        pose.convertTo(pose, CV_32FC1);
        cv::Mat poseInv = pose.inv();
        cv::Point2f posi(poseInv.at<float>(0,3),poseInv.at<float>(2,3));
        cv::Point2f posi_px =posi-estPosi+cv::Point2f(200,200);
        if (!(posi_px.x>=0 && posi_px.x<400 && posi_px.y>=0 && posi_px.y<400)){
            continue;
        }
        Vector6d se3 = getLie(pose);
        all_pose_Lie.push_back(se3);
        all_pose_M.push_back(pose.clone());
        all_pose.push_back(posi);
        std::vector<int> temp_i_vec;
        temp_i_vec.push_back(choose[0]);
        temp_i_vec.push_back(choose[1]);
        temp_i_vec.push_back(choose[2]);
        temp_i_vec.push_back(choose[3]);
        groups.push_back(temp_i_vec);
        img.at<unsigned char>((int)posi_px.x, (int)posi_px.y)=img.at<unsigned char>((int)posi_px.x, (int)posi_px.y)+1;
        //img.at<unsigned char>((int)posi_px.x, (int)posi_px.y)=255;
    }
    std::vector<int> group_vote;
    group_vote.resize(all_pose.size());
    std::vector<std::vector<int>> group_vote_can;
    group_vote_can.resize(all_pose.size());
    for(int i=0;i<all_pose.size();i++){
        for(int j=0;j<all_pose.size();j++){
            if (i==j){
                continue;
            }
            cv::Point2f temp= all_pose[i]-all_pose[j];
            float dis =sqrt(temp.x*temp.x+temp.y*temp.y);
            if(dis <1){
                group_vote[i]++;
                group_vote_can[i].push_back(j);
            }
        }
    }
    int max_vote=-1;
    int max_vote_id=-1;
    for(int i=0; i<group_vote.size();i++){
        if (max_vote < group_vote[i]){
            max_vote = group_vote[i];
            max_vote_id = i;
        }
    }
    std::vector<cv::Mat> pose_list;
    std::vector<int> pair_vote_count;
    pair_vote_count.resize(mps.size());
    if(max_vote_id ==-1){
        return -1;
    }
    for (int i=0;i<group_vote_can[max_vote_id].size();i++){
        pose_list.push_back(all_pose_M[group_vote_can[max_vote_id][i]]);
        for (int j=0;j<4;j++){
            int pair_ind = groups[group_vote_can[max_vote_id][i]][j];
            pair_vote_count[pair_ind]= pair_vote_count[pair_ind]+1;
        }
    }
//    std::vector<float> pair_vote_count_norm;
//    for( int j = 0; j < pair_vote_count.size(); j++ )
//    {
//        if(pair_used_count[j]<20){
//            pair_vote_count_norm.push_back(0);
//        }else{
//            pair_vote_count_norm.push_back(pair_vote_count[j]/(float)pair_used_count[j]);
//        }
//        
//    }
//    std::vector<float> pair_vote_count_norm_sort = pair_vote_count_norm;
//    std::sort(pair_vote_count_norm_sort.begin(), pair_vote_count_norm_sort.end());
//    float max_voted = pair_vote_count_norm_sort.back();
//    float vote_thres = max_voted*0.1;
    for( int j = 0; j < pair_vote_count.size(); j++ )
    {
        if (pair_vote_count[j]>3){
            inliers.insert(j);
        }
    }
    cv::Mat centerM = avi_pose(pose_list);
    std::cout<<"inlier count: "<<inliers.size()<<std::endl;
    double max_in_img;
    double min_in_img;
    cv::minMaxLoc(img, &min_in_img, &max_in_img);
    img=img*(255/(float)max_in_img);
//    cv::imshow("image", img);
//    cv::waitKey(0.01);
    re= centerM;
    return 1;
}

void matchDesc(cv::Mat &descs1, cv::Mat &descs2, std::vector<cv::DMatch> &good_matches){
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descs1, descs2, matches);
    std::sort(matches.begin(), matches.end());
    const int ptsPairs = std::min(1000, (int)(matches.size() * 0.5f));
    for( int j = 0; j < ptsPairs; j++ )
    {
        good_matches.push_back(matches[j]);
    }
}

void addViewMp(chamo::ViewInterfaceImp* viewerHandler, int channel, int mpId, cv::Mat &mp){
    chamo::MapPointCpp item;
    item.id = mpId ;
    item.x = mp.at<float>(0);
    item.y = mp.at<float>(1);
    item.z = mp.at<float>(2);
    viewerHandler->AddMapPoint(channel,&item);
}

void addViewKF(chamo::ViewInterfaceImp* viewerHandler, int channel, int frameId, cv::Mat &centerM){
    cv::Mat centerMInv = centerM;
    chamo::KeyFrameCpp item;
    item.id = frameId;
    item.x = centerMInv.at<float>(0,3);
    item.y = centerMInv.at<float>(1,3);
    item.z = centerMInv.at<float>(2,3);
    item.r00 = centerMInv.at<float>(0,0);
    item.r01 = centerMInv.at<float>(0,1);
    item.r02 = centerMInv.at<float>(0,2);
    item.r10 = centerMInv.at<float>(1,0);
    item.r11 = centerMInv.at<float>(1,1);
    item.r12 = centerMInv.at<float>(1,2);
    item.r20 = centerMInv.at<float>(2,0);
    item.r21 = centerMInv.at<float>(2,1);
    item.r22 = centerMInv.at<float>(2,2);
    viewerHandler->AddKeyFrame(channel,&item);
}
#endif
