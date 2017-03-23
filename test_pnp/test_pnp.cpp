/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   mono_kitti.cpp
 * @brief  Example of using Monoculae sensor SLAM system. This example is tested
 *         vehicle side algorithm.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.03.14        Ming Chen      Create
 *      2016.04.01        Zili Wang      This is the simplest example of using vehicle library
 *******************************************************************************
 */

#include <iostream>
#include <string>
#include "AlgoInterfaceImp.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "DbgViewer.h"
#include "DbgMapDrawer.h"
#include "ORBextractor.h"
#include "Config.h"
#include "SGDmatcher.h"
#include "OrbMap.h"
#include "visualisation/ViewInterfaceImp.h"
#include "se3.hpp"
#include <ctime>

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

void genLocKF(algo::AlgoRTMatrixIn &rts, algo::AlgoKeyFrameIn &kfs, LocData &locData){
    kfs.getKFs(locData.vKFD_);
    kfs.getMPs(locData.vMP3D_);
}
typedef Eigen::Matrix<double,6,1> Vector6d;
cv::Mat getCenter(cv::Mat se3s)
{
    int PsNum = se3s.rows;
    cv::Mat distance_matrix = cv::Mat::zeros(PsNum,PsNum, CV_64FC1);
    for (int i = 0; i<PsNum; i++){
        for (int j = 0; j<PsNum; j++){
            if (i==j){
                distance_matrix.at<double>(i,j) = 9999;
            }else{
                distance_matrix.at<double>(i,j) = cv::norm(se3s.row(i)-se3s.row(j));
            }
            
        }
    }
//    std::vector<double> distance_order = unique(distance_matrix);
//    int centerInd = round(distance_order.size()/4);
//    double distance_median = distance_order[centerInd];
    double distance_median = 30;
    for (int i = 0; i<PsNum; i++){
        for (int j = 0; j<PsNum; j++){
            if (distance_matrix.at<double>(i,j) > distance_median){
                distance_matrix.at<double>(i,j) = -1;
            }
        }
    }
    
    cv::Mat num_point_near = cv::Mat::zeros(PsNum,1, CV_64FC1);
    
    for (int i = 0; i<PsNum; i++){
        int countPos=0;
        for(int j=0;j<distance_matrix.cols;j++){
            if (distance_matrix.at<double>(i,j)>0){
                countPos++;
            }
        }
        num_point_near.at<double>(i, 0) = countPos;
    }
    
    cv::Mat sortIndx;
    cv::sortIdx(num_point_near, sortIndx,CV_SORT_EVERY_COLUMN+CV_SORT_DESCENDING);
    sortIndx.convertTo(sortIndx, CV_32SC1);
//    std::cout<<num_point_near.t()<<std::endl;
//    std::cout<<sortIndx.t()<<std::endl;
//    std::cout<<se3s.row(sortIndx.at<int>(0,0))<<std::endl;
    int bestCondInd = 0;
    for (int i=0;i<sortIndx.rows;i++){
        if (cv::norm(se3s.row(sortIndx.at<int>(i,0)).colRange(0, 3))>1){
            bestCondInd = i;
            break;
        }
    }
    std::vector<int> candidate_index;
    for(int j=0;j<distance_matrix.cols;j++){
        if (distance_matrix.at<double>(sortIndx.at<int>(bestCondInd,0),j)>0){
            candidate_index.push_back(j);
        }
    }
    
    cv::Mat center(se3s.cols,1,CV_64FC1);
    cv::Mat con_se3s(candidate_index.size(), se3s.cols, CV_64FC1);
    
    for(int j=0;j<candidate_index.size();j++){
        int ind = candidate_index[j];
        se3s.row(ind).copyTo(con_se3s.row(j));
    }
//    cv::Mat reInt;
//    distance_matrix.convertTo(reInt, CV_32SC1);
//    std::cout<<reInt<<std::endl;
    //std::cout<<con_se3s<<std::endl;
    for (int i = 0; i<con_se3s.cols; i++){
        
        center.at<double>(i,0) = cv::mean(con_se3s.col(i))[0];
    }
    std::cout<<center.t()<<std::endl;
    return center;
}

void plotParam(cv::Mat data){
    cv::Mat meanX, meanY, stdX, sedY;
    cv::meanStdDev(data, meanX, stdX);
    std::cout<<meanX<<std::endl;
    
//    for (int i=0;i<data.rows;i++){
//        
//    }
}

int main(int argc, char **argv)
{
    std::srand(0);
    time_t timer_extract1, timer_extract2;
    time_t timer_match1, timer_match2;
    time_t timer_pnp1, timer_pnp2;
    std::string cfg_path = argv[1];
    
    algo::AlgoSlamConfig *pISlamCfg = nullptr;
    algo::AlgoCamParam *pICamParam = nullptr;
    pISlamCfg = new algo::AlgoSlamConfig(cfg_path);
    pICamParam = new algo::AlgoCamParam(cfg_path);
    int nFeatures = pISlamCfg->getFeatures();
    float fScaleFactor = pISlamCfg->getScaleFactor();
    int nLevels = pISlamCfg->getLevels();
    int scoreType = pISlamCfg->getScoreType();
    int fIniThFAST = pISlamCfg->getIniThFAST();
    int fMinThFAST = pISlamCfg->getMinThFAST();
    double qualityLevel = pISlamCfg->getSUSANQualityLevel();
    double minDistance = pISlamCfg->getSUSANMinDistance();
    std::string binRoot = "/Volumes/chamo/dataset/gm_new/slam";
    std::string traj1_name= "2016-11-21_T_21-10-41.247_GMT_cut3";
    algo::AlgoRTMatrixIn *pIRTMat1 = new algo::AlgoRTMatrixIn();
    algo::AlgoKeyFrameIn *pIKFs1 = new algo::AlgoKeyFrameIn();
    std::string tracking_name= "2016-11-21_T_21-10-41.247_GMT_cut3";
    chamo::ViewInterfaceImp* viewerHandler = new chamo::ViewInterfaceImp();
    
    algo::SlamData traj1;
    pIRTMat1->loadRTMatrixData(binRoot + "/" +traj1_name + "/RTMatrix_data_0.bin");
    pIKFs1->loadKeyFrameData(binRoot + "/" +traj1_name + "/keyframe_data_0.bin");
    int frameId = 720;
    
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = pICamParam->fx();
    K.at<float>(1,1) = pICamParam->fy();
    K.at<float>(0,2) = pICamParam->cx();
    K.at<float>(1,2) = pICamParam->cy();
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = pICamParam->k1();
    DistCoef.at<float>(1) = pICamParam->k2();
    DistCoef.at<float>(2) = pICamParam->p1();
    DistCoef.at<float>(3) = pICamParam->p2();
    cv::Point3f relGps;
    algo::SGDConfig::getInstance()->init();
    ORBVocabulary* mpVocabulary = new ORBVocabulary();
    ORB_SLAM2::ORBextractor *extractor = new ORBextractor("",nFeatures,fScaleFactor,nLevels,scoreType,fIniThFAST,fMinThFAST,qualityLevel,minDistance);
//    algo::OrbMap *mpMap = algo::OrbMap::Instance();
//    KeyFrameDatabase* mpKeyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*mpVocabulary);
    
    std::vector<algo::MapPoint3D> MPs;
    pIKFs1->getMPs(MPs);
    for (int i=0;i<MPs.size();i++){
        cv::Mat mp= MPs[i].pos_;
        chamo::MapPointCpp item;
        item.id = i ;
        item.x = mp.at<float>(0);
        item.y = mp.at<float>(1);
        item.z = mp.at<float>(2);
        viewerHandler->AddMapPoint(0,&item);
    }
    cv::Mat descs2(MPs.size(), 64, CV_32FC1);
    for (int i=0;i<MPs.size();i++){
        MPs[i].descriptor_.copyTo(descs2.row(i));
    }
    for(int frameId=100; frameId<1500;frameId=frameId+30){
    //for(int frameId=220; frameId<221;frameId=frameId+30){
        std::cout<<frameId<<std::endl;
        cv::Mat img1;
        timer_extract1 = std::clock();
        img1 = readImg(tracking_name,frameId);
        algo::OrbFrame mCurrentFrame(frameId, img1,0.0f,relGps,extractor, mpVocabulary,K,DistCoef ,0.0f,0.0f);
        timer_extract2 = std::clock();
        timer_match1 = timer_extract2;
        cv::BFMatcher matcher;
        std::vector<cv::DMatch> matches;
        matcher.match(mCurrentFrame.mDescriptors, descs2, matches);
        std::sort(matches.begin(), matches.end());
        std::vector<cv::DMatch> good_matches;
        const int ptsPairs = std::min(1000, (int)(matches.size() * 0.1f));
        for( int j = 0; j < ptsPairs; j++ )
        {
            good_matches.push_back(matches[j]);
        }
        timer_match2 = std::clock();
        timer_pnp1 = timer_match2;
        int pairCount=100;
        cv::Mat se3s(pairCount, 6, CV_64FC1);
        for(int j=0;j<pairCount;j++){
            cv::Mat rvec;
            cv::Mat tvec;
            std::vector<cv::Point3f> objectPoints;
            std::vector<cv::Point2f> imagePoints;
            
            for (int i=0;i<4;i++){
                int random_variable = std::rand();
                random_variable=round(random_variable/(float)RAND_MAX*50);
                cv::Point3f p3f;
                cv::Point2f p2f = mCurrentFrame.mvKeysUn[good_matches[random_variable].queryIdx].pt;
                cv::Mat mp= MPs[good_matches[random_variable].trainIdx].pos_;
                p3f.x = mp.at<float>(0);
                p3f.y = mp.at<float>(1);
                p3f.z = mp.at<float>(2);
                objectPoints.push_back(p3f);
                imagePoints.push_back(p2f);
            }
            cv::solvePnP(objectPoints, imagePoints, K, DistCoef, rvec, tvec, false, CV_P3P);
            cv::Mat rotM;
            cv::Rodrigues(rvec, rotM);
            cv::Mat pose = cv::Mat::eye(4, 4, CV_64FC1);
            rotM.copyTo(pose.colRange(0, 3).rowRange(0, 3));
            tvec.copyTo(pose.col(3).rowRange(0, 3));
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
            Eigen::Vector3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            Sophus::SE3d cSE3(ro_eigen, t);
            Vector6d se3=cSE3.log();
            for (int i=0;i<6;i++){
                se3s.at<double>(j,i) =se3(i,0);
            }
            //std::cout<<se3.transpose()<<std::endl;
        }
        cv::Mat center = getCenter(se3s);
        Vector6d center6d;
        for (int i=0;i<6;i++){
            center6d(i,0) =center.at<double>(i,0);
        }
        Sophus::SE3d centerSE3 = Sophus::SE3d::exp(center6d);
        
        
        Sophus::SE3d::Transformation poseTran = centerSE3.matrix();
        cv::Mat centerM(4,4,CV_64FC1);
        for (int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                centerM.at<double>(i,j)=poseTran(i,j);
            }
        }
        timer_pnp2 = std::clock();
        printf ("extra_time:%f ",((float)timer_extract2- timer_extract1)/CLOCKS_PER_SEC);
        printf ("match_time:%f ",((float)timer_match2- timer_match1)/CLOCKS_PER_SEC);
        printf ("pnp_time:%f\n",((float)timer_pnp2-timer_pnp1)/CLOCKS_PER_SEC);
        if (cv::norm(centerM.col(3).rowRange(0, 3))<1){
            continue;
        }
        //std::cout<<centerSE3.matrix()<<std::endl;
        cv::Mat centerMInv = centerM.inv();
        chamo::KeyFrameCpp item;
        item.id = frameId;
        item.x = centerMInv.at<double>(0,3);
        item.y = centerMInv.at<double>(1,3);
        item.z = centerMInv.at<double>(2,3);
        item.r00 = centerMInv.at<double>(0,0);
        item.r01 = centerMInv.at<double>(0,1);
        item.r02 = centerMInv.at<double>(0,2);
        item.r10 = centerMInv.at<double>(1,0);
        item.r11 = centerMInv.at<double>(1,1);
        item.r12 = centerMInv.at<double>(1,2);
        item.r20 = centerMInv.at<double>(2,0);
        item.r21 = centerMInv.at<double>(2,1);
        item.r22 = centerMInv.at<double>(2,2);
        viewerHandler->AddKeyFrame(1,&item);
        
    }
    viewerHandler->SaveAll("/Volumes/chamo/dataset/all.chamo");
    
    //std::cout <<invR<<std::endl;
    //std::cout <<tvec<<std::endl;
    
//    for(int i=0; i< mCurrentFrame.mvKeysUn.size();i++){
//        cv::circle(img1, mCurrentFrame.mvKeysUn[i].pt, 3, cv::Scalar(255, 0, 0));
//    }
//    viewerHandler->SaveAll("/Volumes/chamo/dataset/all.chamo");
//    cv::Size size;
//    size.height =img1.rows*0.4;
//    size.width =img1.cols*0.4;
//    cv::resize(img1, img2, size);
//    cv::imshow("img1", img2);
//    cv::waitKey(-1);
    return 0;
}

