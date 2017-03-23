#ifndef customView_h
#define customView_h

#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import "OpenGLView.h"
#import <CoreMotion/CoreMotion.h>
#include <vector>
#include <opencv2/opencv.hpp>
#import <AVFoundation/AVFoundation.h>

@interface customView : UIViewController<AVCapturePhotoCaptureDelegate>{
    std::vector<cv::Point3f> atitudes;
    std::vector<cv::Point3f> acces;
    std::vector<cv::Point3f> rotRates;
    bool need_record;
    std::string file_name;
    long long last_time;
    cv::Point3f real_acce;
    cv::Point3f last_acce;
    cv::Point3f cur_v;
    cv::Point3f cur_posi;
    bool first_frame;
    int render_count;
    bool cal_translation;
    bool photo_cam_off;
    AVCapturePhotoOutput *photo_output;
    std::vector<std::vector<cv::Point2f> > cali_imagePoints;
}
@property (weak, nonatomic) IBOutlet OpenGLView *glView;
@property (weak, nonatomic) IBOutlet UIImageView *imgView;
@property (weak, nonatomic) CMMotionManager *motionManager;
@property (weak, nonatomic) IBOutlet UIButton *RestIMU_btn;
- (IBAction)resetIMU:(id)sender;
@property (weak, nonatomic) IBOutlet UIImageView *re_img_view;
@property (weak, nonatomic) IBOutlet UIButton *cali_btn;
- (IBAction)cali_all:(id)sender;
@property (weak, nonatomic) IBOutlet UILabel *console1;
@property (weak, nonatomic) IBOutlet UILabel *console2;
@property (weak, nonatomic) IBOutlet UILabel *console3;
@property (weak, nonatomic) IBOutlet UILabel *console4;



@property (weak, nonatomic) IBOutlet UIButton *video_btn;
- (IBAction)start_video:(id)sender;

- (IBAction)start_re:(id)sender;
- (IBAction)end_re:(id)sender;
@property (weak, nonatomic) IBOutlet UIButton *start_re_btn;
@property (weak, nonatomic) IBOutlet UIButton *end_re_btn;
@end
#endif /* customView_h */
