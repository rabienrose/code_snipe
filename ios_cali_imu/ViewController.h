//
//  ViewController.h
//  orbslamios
//
//  Created by Ying Gaoxuan on 16/11/1.
//  Copyright © 2016年 Ying Gaoxuan. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "OpenGLView.h"
#import <CoreMotion/CoreMotion.h>
#import "orbslamios-Swift.h"
#import "customView.h"

@interface ViewController : UITabBarController{
    DeviceMotionViewController *motionView;
    MagnetometerViewController *magentView;
    GyroscopeViewController *gyroView;
    AccelerometerViewController *acceleroView;
    customView *myView;
    NSMutableData *_responseData;
}
//@property (weak, nonatomic) IBOutlet UIButton *start;
//@property (nonatomic, strong) CvVideoCamera* videoCamera;
//@property UIImageView *frameLabel;

@end
