#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <iomanip>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[]) {
//    std::string img1_name = "/Volumes/chamo/dataset/cap_20170323061404/image/100.jpg";
//    std::string img2_name = "/Volumes/chamo/dataset/cap_20170323061404/image/110.jpg";
    std::string img1_name = "/Volumes/chamo/working/gazebo/recording/image/img_000030.jpg";
    std::string img2_name = "/Volumes/chamo/working/gazebo/recording/image/img_000050.jpg";
    cv::Mat img1 = cv::imread(img1_name);
    cv::Mat img2 = cv::imread(img2_name);
    //cv::flip(img1, img1, 0);
    //cv::flip(img2, img2, 0);
    //cv::resize(img1, img1, cv::Size(img1.cols/2, img1.rows/2));
    //cv::resize(img2, img2, cv::Size(img2.cols/2, img2.rows/2));
    cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    std::sort(matches.begin(), matches.end());
    std::vector<cv::DMatch> good_matches;
    const int ptsPairs = std::min(1000, (int)(matches.size() * 0.2f));
    for( int j = 0; j < ptsPairs; j++ )
    {
        good_matches.push_back(matches[j]);
    }
    
    std::vector<cv::Point2f> points1(good_matches.size());
    std::vector<cv::Point2f> points2(good_matches.size());
    std::vector<int> points1_ind(good_matches.size());
    std::vector<int> points2_ind(good_matches.size());
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        points1_ind[i]=kpInd1;
        points2_ind[i]=kpInd2;
        points1[i] =keypoints1[kpInd1].pt;
        points2[i] =keypoints2[kpInd2].pt;
    }
    cv::Mat inliers;
    findFundamentalMat(points1, points2, cv::FM_RANSAC, 3., 0.99, inliers);
    //std::cout<<inliers.type()<<std::endl;
    
    std::vector<cv::DMatch> good_matches_raw;
    good_matches_raw =good_matches;
    good_matches.clear();
    for( int j = 0; j < inliers.rows; j++ )
    {
        if(inliers.at<unsigned char>(j)==1){
            good_matches.push_back(good_matches_raw[j]);
        }
    }
    
    cv::Mat track_canvas;
    img2.copyTo(track_canvas);
    for(int i=0; i< keypoints2.size();i++){
        cv::circle(track_canvas, keypoints2[i].pt, 1, cv::Scalar(0,0,0),2);
    }
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        cv::line(track_canvas, keypoints1[kpInd1].pt, keypoints2[kpInd2].pt, CV_RGB(255,0,0));
        cv::circle(track_canvas, keypoints2[kpInd2].pt, 1, CV_RGB(255,0,0),2);
    }
    cv::imshow("track_canvas", track_canvas);
    
    cv::Size size;
    size.height =img1.rows;
    size.width =img1.cols;
    cv::Mat canvas(size.height*2, size.width, img1.type());
    img1.copyTo(canvas.rowRange(0, size.height));
    img2.copyTo(canvas.rowRange(size.height, size.height*2));
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        cv::Point2f siftpt = keypoints2[kpInd2].pt;
        siftpt.y=siftpt.y+size.height;
        cv::line(canvas, keypoints1[kpInd1].pt, siftpt, CV_RGB(255,255,255));
        
        cv::Scalar clr(rand() % 255,rand() % 255,rand() % 255);
        cv::circle(canvas, keypoints1[kpInd1].pt, 2, clr,3);
        cv::circle(canvas, siftpt, 2, clr,3);
    }
    cv::Size sizeCan;
    sizeCan.height =canvas.rows*1;
    sizeCan.width =canvas.cols*1;
    cv::resize(canvas, canvas, sizeCan);
    cv::imshow("img1", canvas);
    
    cv::waitKey(-1);
}
