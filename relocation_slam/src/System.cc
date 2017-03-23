#include "System.h"
#include "Converter.h"
#include <iomanip>
#include <unistd.h>
#include"ORBmatcher.h"
#include "opencv2/calib3d.hpp"

namespace ORB_SLAM2
{
    
    System::System(const string &strVocFile, const string &strSettingsFile)
    {
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }
        
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        mpMap = new Map();
        mpTracker = new Tracking(this, mpVocabulary, mpMap, mpKeyFrameDatabase, strSettingsFile);
        mpLocalMapper = new LocalMapping(mpMap);
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary);
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }
    
    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
    {
        cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }
    
    void showMatch(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat img1, cv::Mat img2){
        cv::Size size;
        size.height =img1.rows;
        size.width =img1.cols;
        
        cv::Mat canvas(size.height*2, size.width, img1.type());
        img1.copyTo(canvas.rowRange(0, size.height));
        img2.copyTo(canvas.rowRange(size.height, size.height*2));
        for(int i=0; i< pts1.size();i++){
            cv::Point2f siftpt = pts2[i];
            siftpt.y=siftpt.y+size.height;
            //cv::line(canvas, pts1[i], siftpt, CV_RGB(255,255,255));
            cv::Scalar clr(rand() % 255,rand() % 255,rand() % 255);
            cv::circle(canvas, pts1[i], 3, clr,5);
            cv::circle(canvas, siftpt, 3, clr,5);
        }
        cv::Size sizeCan;
        sizeCan.height =canvas.rows*0.5;
        sizeCan.width =canvas.cols*0.5;
        cv::resize(canvas, canvas, sizeCan);
        cv::imshow("2d_match", canvas);
    }
    
    void showTrack(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, cv::Mat img2){
        for(int i=0; i< pts1.size();i++){
            cv::Point2f siftpt = pts2[i];
            cv::line(img2, pts1[i], siftpt, CV_RGB(0,0,255));
            cv::circle(img2, siftpt, 1, CV_RGB(255,0,0),2);
        }
        cv::imshow("track", img2);
    }
    
    void showRaw(std::vector<cv::Point2f> pts1, cv::Mat img2, std::string wndName){
        for(int i=0; i< pts1.size();i++){
            cv::Point2f siftpt = pts1[i];
            cv::circle(img2, siftpt, 1, CV_RGB(255,0,0),2);
        }
        cv::imshow(wndName, img2);
    }

    
    void System::VisMatch2d(cv::Mat img1, cv::Mat img2, std::string strSettingPath){
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        
        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];
        cv::cvtColor(img1, img1, CV_BGR2GRAY);
        cv::cvtColor(img2, img2, CV_BGR2GRAY);
        ORBextractor *myORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        Frame frame1(img1,0,myORBextractor,mpVocabulary,K,DistCoef,0, 0.0);
        Frame frame2(img2,0,myORBextractor,mpVocabulary,K,DistCoef,0, 0.0);
        Initializer* myInitializer =  new Initializer(frame1,1.0,4000);
        std::vector<cv::Point2f> tPrevMatched;
        tPrevMatched.resize(frame1.mvKeysUn.size());
        for(size_t i=0; i<frame1.mvKeysUn.size(); i++){
            tPrevMatched[i]=frame1.mvKeysUn[i].pt;
        }
        std::vector<int> myMatches;
        myMatches.resize(frame1.mvKeysUn.size());
        fill(myMatches.begin(),myMatches.end(),-1);
        
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(frame1,frame2,tPrevMatched,myMatches,100);
        cv::BFMatcher matcher_cv;
        std::vector<cv::DMatch> matches;
        matcher_cv.match(frame1.mDescriptors, frame2.mDescriptors, matches);
        std::sort(matches.begin(), matches.end());
        const int ptsPairs = std::min(1000, (int)(matches.size() * 0.1f));
        std::vector<int> myMatches_cv;
        myMatches_cv.resize(frame1.mvKeysUn.size());
        fill(myMatches_cv.begin(),myMatches_cv.end(),-1);
        for( int j = 0; j < ptsPairs; j++ )
        {
            myMatches_cv[matches[j].queryIdx]=matches[j].trainIdx;
        }

        cv::cvtColor(img1, img1, CV_GRAY2BGR);
        cv::cvtColor(img2, img2, CV_GRAY2BGR);
        std::vector<cv::Point2f> pts1;
        std::vector<cv::Point2f> pts2;
        for (int i=0; i<myMatches_cv.size();i++){
            if(myMatches_cv[i]>0){
                pts1.push_back(frame1.mvKeysUn[i].pt);
                pts2.push_back(frame2.mvKeysUn[myMatches_cv[i]].pt);
            }
        }
        showRaw(pts1,img1.clone(), "img1");
        showRaw(pts2,img2.clone(), "img2");
        showTrack(pts1, pts2, img2.clone());
        showMatch(pts1, pts2, img1, img2);
        cv::waitKey(-1);
        delete myORBextractor;
        delete myInitializer;
    }
    
    int System::GetTrackingState()
    {
        return mTrackingState;
    }
    
    vector<MapPoint*> System::GetTrackedMapPoints()
    {
        return mTrackedMapPoints;
    }
    
    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        return mTrackedKeyPointsUn;
    }
    
} //namespace ORB_SLAM
