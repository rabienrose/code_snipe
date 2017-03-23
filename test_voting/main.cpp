#include "main_helper.h"
#include "pose_voting.h"
#include <iostream>
#include <fstream>

cv::Point3f stand_point(-83.682875,42.518849,2.800000);

std::vector<std::string> split(std::string str, char delimiter) {
    vector<string> internal;
    stringstream ss(str); // Turn the string into a stream.
    string tok;
    
    while(getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }
    
    return internal;
}

cv::Point2f gps2coor(cv::Point3f input)
{
    double diff0 = input.x - stand_point.x;
    double diff1 = input.y - stand_point.y;
    
    double latitude = stand_point.x * 3.14159 / 180;
    
    cv::Point2f re;
    re.x = diff0*(111413*std::cos(latitude) - 94*std::cos(3*latitude));
    re.y = diff1*111320;

    return re;
}

void readAGps(std::string filename, std::vector<cv::Point2f>& gps_posi){
    std::ifstream gps_file(filename);
    while(true){
        std::string str;
        std::getline(gps_file, str);
        if(str.empty()){
            break;
        }
        std::vector<std::string> sep = split(str,',');
        if(sep.size()<=1){
            continue;
        }
        cv::Point3f t_posi;
        t_posi.x = std::stof(sep[0].c_str());
        t_posi.y = std::stof(sep[1].c_str());
        t_posi.z = std::stof(sep[2].c_str());
        cv::Point2f re = gps2coor(t_posi);
        gps_posi.push_back(re);
    }
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
    std::string root_str = "/Volumes/chamo/dataset/gm_new/";
    std::string binRoot = root_str + "/slam";
    std::string mp_name= "2016-11-21_T_21-10-41.247_GMT_cut3";
    algo::AlgoRTMatrixIn *pIRTMat_mp = new algo::AlgoRTMatrixIn();
    algo::AlgoKeyFrameIn *pIKFs_mp = new algo::AlgoKeyFrameIn();
    algo::AlgoRTMatrixIn *pIRTMat_track = new algo::AlgoRTMatrixIn();
    algo::AlgoKeyFrameIn *pIKFs_track = new algo::AlgoKeyFrameIn();
    std::string tracking_name= "2016-11-24_T_21-33-34.499_GMT_cut4";
    chamo::ViewInterfaceImp* viewerHandler = new chamo::ViewInterfaceImp();
    
    algo::SlamData traj1;
    pIRTMat_mp->loadRTMatrixData(binRoot + "/" +mp_name + "/RTMatrix_data_0.bin");
    pIKFs_mp->loadKeyFrameData(binRoot + "/" +mp_name + "/keyframe_data_0.bin");
    pIRTMat_track->loadRTMatrixData(binRoot + "/" +tracking_name + "/RTMatrix_data_0.bin");
    pIKFs_track->loadKeyFrameData(binRoot + "/" +tracking_name + "/keyframe_data_0.bin");
    
    
    std::vector<cv::Point2f> gps_mp;
    std::vector<cv::Point2f> gps_tracking;
    readAGps(root_str+"/kml/"+mp_name+".kml", gps_mp);
    readAGps(root_str+"/kml/"+tracking_name+".kml", gps_tracking);
    
    
    std::vector<int> kf2f_mp;
    getKF2FMapping(pIRTMat_track, kf2f_mp);
    
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
    pIKFs_mp->getMPs(MPs);
    for (int i=0;i<MPs.size();i++){
        cv::Mat mp= MPs[i].pos_;
        addViewMp(viewerHandler, 1, i, mp);
    }
    cv::Mat descs2(MPs.size(), 64, CV_32FC1);
    for (int i=0;i<MPs.size();i++){
        MPs[i].descriptor_.copyTo(descs2.row(i));
    }
    
//    for(int frameId=100; frameId<700;frameId=frameId+1){
//        cv::Mat poseTrue;
//        bool isKf;
//        pIRTMat1->get(frameId, poseTrue, isKf);
//        poseTrue = poseTrue.inv();
//        float y = poseTrue.at<float>(1,2);
//        float angle = asin(y);
//        //std::cout<<angle/3.11415*180<<std::endl;
//        std::cout<<poseTrue.at<float>(1,3)<<std::endl;
//    }

//    for(int frameId=0; frameId<pIKFs_mp->getKFNum();frameId=frameId+1){
//        cv::Mat poseTrue;
//        algo::KeyFrameData kf;
//        pIKFs_mp->getKF(frameId, kf);
//        cv::Mat TranM = cv::Mat::eye(4,4, CV_32FC1);
//        kf.r.copyTo(TranM.colRange(0, 3).rowRange(0, 3));
//        kf.t.copyTo(TranM.col(3).rowRange(0, 3));
//        cv::Mat TranMInv = TranM.inv();
//        addViewKF(viewerHandler, 0, frameId, TranMInv);
//    }
//    
//    for(int frameId=0; frameId<pIKFs_track->getKFNum();frameId=frameId+1){
//        cv::Mat poseTrue;
//        algo::KeyFrameData kf;
//        pIKFs_track->getKF(frameId, kf);
//        cv::Mat TranM = cv::Mat::eye(4,4, CV_32FC1);
//        kf.r.copyTo(TranM.colRange(0, 3).rowRange(0, 3));
//        kf.t.copyTo(TranM.col(3).rowRange(0, 3));
//        cv::Mat TranMInv = TranM.inv();
//        addViewKF(viewerHandler, 1, frameId, TranMInv);
//    }
    
//    viewerHandler->SaveAll("/Volumes/chamo/dataset/all.chamo");
    
    int frameId = 467;
    int out_count=0;
    for(int frameId=0; frameId<660;frameId=frameId+10){
    //for(int frameId=1090; frameId<1091;frameId=frameId+1){
    //for(float param= 0.1; param<3;param=param+0.2){
        out_count++;

        cv::Point2f tposi;
        int ref_fid;
        if (!getEstPosi(gps_tracking[frameId], gps_mp, tposi, ref_fid)){
            continue;
        }
        bool isKf;
        cv::Mat truePose;
        pIRTMat_mp->get(ref_fid, truePose, isKf);
        if(truePose.type()!= CV_32FC1){
            continue;
        }
        truePose= truePose.inv();
        cv::Point2f rePosi(truePose.at<float>(0,3), truePose.at<float>(2,3));
        std::cout<<"ref frameId: "<<ref_fid<<std::endl;
        std::cout<<"ref gps posi: "<<tposi<<std::endl;
        std::cout<<"cur frameId: "<<frameId<<std::endl;
        std::cout<<"cur gps posi: "<<gps_tracking[frameId]<<std::endl;
        cv::Mat img1;
        //cv::Mat img2;
        timer_extract1 = std::clock();
        img1 = readImg(tracking_name,frameId);
        //img2 = readImg(tracking_name,frameId);
        algo::OrbFrame mCurrentFrame(frameId, img1,0.0f,relGps,extractor, mpVocabulary,K,DistCoef ,0.0f,0.0f);
        timer_extract2 = std::clock();
        timer_match1 = timer_extract2;
        std::vector<cv::DMatch> good_matches;
        matchDesc(mCurrentFrame.mDescriptors, descs2, good_matches);
        for (int n = 0; n<good_matches.size(); n++){
            cv::Mat mp = MPs[good_matches[n].trainIdx].pos_;
            //addViewMp(viewerHandler, 2, n, mp);
        }
        timer_match2 = std::clock();
        timer_pnp1 = timer_match2;
        VotingSpace votingSpace;
        
        float pitchAng = -1/(float)180*3.1415;
        float eer_h= 0.25;
        float eer_r= 15;
        float cam_h=0;
        for (int n = 0; n<good_matches.size(); n++){
            float minD;
            float maxD;
            cv::Mat mp = MPs[good_matches[n].trainIdx].pos_;
            cv::Point3f posi(mp.at<float>(0,0), mp.at<float>(1,0), mp.at<float>(2,0));
            cv::Point2f uv = mCurrentFrame.mvKeysUn[good_matches[n].queryIdx].pt;
            votingSpace.getDepthRange(K, posi, uv, pitchAng,eer_h, cam_h, eer_r, minD, maxD);
            //std::cout<<minD<<":"<<maxD<<"|||"<<posi.x<<":"<<posi.z<<std::endl;
            if (minD<0 || maxD<0 || minD>10000 || maxD>10000){
                continue;
            }
            votingSpace.gps =rePosi;
            votingSpace.addErrShape(posi, minD, maxD, n);
        }
        std::set<int> candi;
        cv::Mat img = votingSpace.visVotingSpace(candi);
        std::string imageRoot = "/Volumes/chamo/document/voting_out/temp/";
        std::stringstream ss;
        ss<<imageRoot<< std::setw (4)<< std::setfill ('0')<<out_count<<".jpg";
        cv::imwrite(ss.str(), img);
//        cv::imshow("Pose Voting", img);
//        cv::waitKey(0.01);
        
        
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        std::set<int>::iterator it;
        for (it=candi.begin(); it!=candi.end(); ++it){
            cv::Mat mp = MPs[good_matches[*it].trainIdx].pos_;
            //addViewMp(viewerHandler, 3, 0, mp);
            cv::Point3f posi(mp.at<float>(0,0), mp.at<float>(1,0), mp.at<float>(2,0));
            cv::Point2f uv = mCurrentFrame.mvKeysUn[good_matches[*it].queryIdx].pt;
            objectPoints.push_back(posi);
            imagePoints.push_back(uv);
            
        }
        cv::Mat rvec;
        cv::Mat tvec;
        std::set<int> inliers_set;
        cv::Mat est_pose;

        
        int re = doPnp(objectPoints, imagePoints, K, DistCoef, est_pose, inliers_set, rePosi);
        if (re==-1){
            continue;
        }
        
        std::vector<cv::Point3f> objectPoints3;
        std::vector<cv::Point2f> imagePoints3;
        std::set<int>::iterator it_int_set2;
        for (it_int_set2 = inliers_set.begin(); it_int_set2 != inliers_set.end(); ++it_int_set2)
        {
            int id = *it_int_set2; // Note the "*" here
            objectPoints3.push_back(objectPoints[id]);
            imagePoints3.push_back(imagePoints[id]);
        }
        std::set<int> inliers_set3;
        re = doPnp(objectPoints3, imagePoints3, K, DistCoef, est_pose, inliers_set3, rePosi);
        if (re==-1 || inliers_set.size()<10){
            continue;
        }
        cv::Mat est_pose_inv=est_pose.inv();
        addViewKF(viewerHandler, 2, frameId, est_pose_inv);
        std::cout<<est_pose_inv.col(3).rowRange(0, 3).t()<<std::endl;
        std::vector<cv::Point3f> objectPoints2;
        std::vector<cv::Point2f> imagePoints2;
        std::set<int>::iterator it_int_set;
        for (it_int_set = inliers_set3.begin(); it_int_set != inliers_set3.end(); ++it_int_set)
        {
            int id = *it_int_set; // Note the "*" here
            objectPoints2.push_back(objectPoints[id]);
            imagePoints2.push_back(imagePoints[id]);
        }
        
        std::vector<int> inliers;
        cv::solvePnPRansac(objectPoints, imagePoints, K, DistCoef, rvec, tvec, false, 600, 5.0, 0.99, inliers, CV_ITERATIVE);
        cv::Mat rotM;
        cv::Rodrigues(rvec, rotM);
        cv::Mat TranM = cv::Mat::eye(4,4, CV_32FC1);
        rotM.copyTo(TranM.colRange(0, 3).rowRange(0, 3));
        tvec.copyTo(TranM.col(3).rowRange(0, 3));
        cv::Mat TranMInv = TranM.inv();
        std::cout<<"ransac inlier: "<<inliers.size()<<std::endl;
        addViewKF(viewerHandler, 1, frameId, TranMInv);
        
        
        cv::Mat truePoseInv=  truePose.inv();
//        std::cout<<"inler count: "<<inliers.size()<<std::endl;
//        for(int i=0;i<imagePoints.size() ;i++){
//            cv::circle(img1, imagePoints[i], 2, cv::Scalar(255, 0, 0, 255), 2);
//        }
//        for(int i=0;i<inliers.size() ;i++){
//            cv::Scalar clr(rand() % 255,rand() % 255,rand() % 255);
//            cv::circle(img1, imagePoints[inliers[i]], 2, cv::Scalar(0, 255, 0, 255),2);
//            //std::cout<<imagePoints[inliers[i]]<<std::endl;
//            cv::Mat posi(4,1,CV_32FC1);
//            cv::Point3f posi_cv = objectPoints[inliers[i]];
//            posi.at<float>(0,0)=posi_cv.x;
//            posi.at<float>(1,0)=posi_cv.y;
//            posi.at<float>(2,0)=posi_cv.z;
//            posi.at<float>(3,0)=1;
//            //addViewMp(viewerHandler, 4, 0, posi);
//            cv::Mat ptHomo = K*TranM.rowRange(0, 3)*posi;
//            float repX = ptHomo.at<float>(0)/ptHomo.at<float>(2);
//            float repY = ptHomo.at<float>(1)/ptHomo.at<float>(2);
//            cv::circle(img1, cv::Point(repX, repY), 2, cv::Scalar(0, 0, 255, 255), 2);
//            cv::line(img1, imagePoints[inliers[i]], cv::Point(repX, repY), cv::Scalar(0, 0, 255, 255));
//        }
//        cv::imshow("image", img1);
//        cv::waitKey(0.01);
        
//        std::cout<<TranMInv.col(3).rowRange(0, 3).t()<<std::endl;
//        std::cout<<truePose.col(3).rowRange(0, 3).t()<<std::endl;
        
        addViewKF(viewerHandler, 0, frameId, truePose);
    }
    viewerHandler->SaveAll("/Volumes/chamo/dataset/all.chamo");
    return 0;
}

