//
//  ViewController.m
//  orbslamios
//
//  Created by Ying Gaoxuan on 16/11/1.
//  Copyright © 2016年 Ying Gaoxuan. All rights reserved.
//

#import "ViewController.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <iomanip>
#include<opencv2/opencv.hpp>

#include "System.h"
#include "MapPoint.h"
#include "common_header.h"
#include <unistd.h>

using namespace cv;
using namespace std;

const char *ORBvoc = [[[NSBundle mainBundle]pathForResource:@"ORBvoc" ofType:@"bin"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
const char *settings = [[[NSBundle mainBundle]pathForResource:@"foun" ofType:@"yaml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];

NSString *newPath;

ORB_SLAM2::System SLAM(ORBvoc,settings);


@implementation ViewController

CMMotionManager *motionManager;

//@synthesize videoCamera = _videoCamera;

- (void)viewDidLoad {
    [super viewDidLoad];
    CGRect screenBounds = [[UIScreen mainScreen] bounds];
    motionManager = [[CMMotionManager alloc] init];
    if (motionManager.deviceMotionAvailable)
    {
        for (UIViewController* pview in self.viewControllers)
        {
            if ([pview isKindOfClass:[UINavigationController class]]){
                UINavigationController *nav_controller = (UIViewController *)pview;
                if ([nav_controller.topViewController isKindOfClass:[DeviceMotionViewController class]]){
                    motionView =  (DeviceMotionViewController*)nav_controller.topViewController;
                    motionView.motionManager =motionManager;
                }
                if ([nav_controller.topViewController isKindOfClass:[MagnetometerViewController class]]){
                    magentView =  (MagnetometerViewController*)nav_controller.topViewController;
                    magentView.motionManager =motionManager;
                }
                if ([nav_controller.topViewController isKindOfClass:[GyroscopeViewController class]]){
                    gyroView =  (GyroscopeViewController*)nav_controller.topViewController;
                    gyroView.motionManager =motionManager;
                }
                if ([nav_controller.topViewController isKindOfClass:[AccelerometerViewController class]]){
                    acceleroView =  (AccelerometerViewController*)nav_controller.topViewController;
                    acceleroView.motionManager = motionManager;
                }
            }else{
                if ([pview isKindOfClass:[customView class]]){
                    myView =(customView* )pview;
                    myView.motionManager = motionManager;
                }
            }
        }
    }
    self.selectedIndex = 4;
    
//    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
//        for (int i=0;i<11;i++){
//            std::stringstream ss;
//            ss<<"images/"<< std::setw (4)<< std::setfill ('0')<<i<<".png";
//            std::string imgAddrStd=ss.str();
//            NSString* imgAddrNs = [NSString stringWithUTF8String:imgAddrStd.c_str()];
//            //NSString* imgAddr=[NSString stringWithFormat:@"%@%@",newPath,imgAddrNs];
//            UIImage* uiImage = [UIImage imageNamed:imgAddrNs];
//            cv::Mat imgMat = [mm_Try cvMatFromUIImage: uiImage];
//            [self processImage: imgMat];
//            
//            dispatch_async(dispatch_get_main_queue(), ^{
//                self.frameLabel.image = [mm_Try UIImageFromCVMat:imgMat];
//                if (SLAM.mpTracker->mCurrentFrame.mnId%1 ==0){
//                    std::vector<MapPointChamo> allmappoints;
//                    std::vector<cv::Mat> allkfs;
//                    const vector<ORB_SLAM2::MapPoint*> vpMPs = SLAM.mpTracker->mpMap->GetAllMapPoints();
//                    if (vpMPs.size() > 0) {
//                        for (size_t i = 0; i < vpMPs.size(); i++) {
//                            if (vpMPs[i] && !vpMPs[i]->isBad())
//                            {
//                                cv::Point3f pos = cv::Point3f(vpMPs[i]->GetWorldPos());
//                                MapPointChamo tMp;
//                                tMp.posi =pos;
//                                allmappoints.push_back(tMp);
//                            }
//                        }
//                        [self.glView SetMapPoints:allmappoints];
//                    }
//                    std::vector<ORB_SLAM2::KeyFrame*> kfs = SLAM.mpTracker->mpMap->GetAllKeyFrames();
//                    if(kfs.size()>0){
//                        for(int i=0;i<kfs.size();i++){
//                            cv::Mat pose =kfs[i]->GetPose();
//                            allkfs.push_back(pose);
//                        }
//                        [self.glView SetKF:allkfs];
//                    }
//                    [self.glView render];
//                }
//            });
//        }
//    });
}

- (void)processImage:(cv::Mat &)image
{
    cv::cvtColor(image,image,CV_BGRA2GRAY);
    cv::Mat pose = SLAM.TrackMonocular(image,0);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    std::vector<cv::KeyPoint>& kps = SLAM.mpTracker->mCurrentFrame.mvKeysUn;
    std::vector<ORB_SLAM2::MapPoint*>& mps = SLAM.mpTracker->mCurrentFrame.mvpMapPoints;
    for (int i=0;i<kps.size();i++){
        cv::circle(image, kps[i].pt, 1, cv::Scalar(255, 0, 0, 255), 2);
        if (mps[i] != NULL){
            cv::Mat posi(4,1,CV_32FC1);
            posi.at<float>(3,0)=1;
            cv::Mat posi_t= mps[i]->GetWorldPos();
            if (posi_t.empty()){
                continue;
            }
            posi_t.copyTo(posi.rowRange(0, 3));
            cv::Mat ptHomo = SLAM.mpTracker->mCurrentFrame.mK*pose.rowRange(0, 3)*posi;
            float repX = ptHomo.at<float>(0)/ptHomo.at<float>(2);
            float repY = ptHomo.at<float>(1)/ptHomo.at<float>(2);
            cv::circle(image, cv::Point(repX, repY), 2, cv::Scalar(0, 255, 0, 255), 3);
            cv::line(image, kps[i].pt, cv::Point(repX, repY),cv::Scalar(0,255,0, 255),2);
        }
    }
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
