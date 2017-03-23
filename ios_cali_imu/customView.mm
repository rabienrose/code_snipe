#import <Foundation/Foundation.h>
#import "customView.h"
#include <iostream>
#include <fstream>
#include "util/img_tool.h"
#include "common_header.h"
#include <TargetConditionals.h>
#if TARGET_RT_BIG_ENDIAN
#   define FourCC2Str(fourcc) (const char[]){*((char*)&fourcc), *(((char*)&fourcc)+1), *(((char*)&fourcc)+2), *(((char*)&fourcc)+3),0}
#else
#   define FourCC2Str(fourcc) (const char[]){*(((char*)&fourcc)+3), *(((char*)&fourcc)+2), *(((char*)&fourcc)+1), *(((char*)&fourcc)+0),0}
#endif
@interface UIImage (fixOrientation)
- (UIImage *)fixOrientation;
@end
@implementation UIImage (fixOrientation)
- (UIImage *)fixOrientation {
    // No-op if the orientation is already correct
    if (self.imageOrientation == UIImageOrientationUp) return self;
    
    // We need to calculate the proper transformation to make the image upright.
    // We do it in 2 steps: Rotate if Left/Right/Down, and then flip if Mirrored.
    CGAffineTransform transform = CGAffineTransformIdentity;
    
    switch (self.imageOrientation) {
        case UIImageOrientationDown:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, self.size.width, self.size.height);
            transform = CGAffineTransformRotate(transform, M_PI);
            break;
            
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
            transform = CGAffineTransformTranslate(transform, self.size.width, 0);
            transform = CGAffineTransformRotate(transform, M_PI_2);
            break;
            
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, 0, self.size.height);
            transform = CGAffineTransformRotate(transform, -M_PI_2);
            break;
        case UIImageOrientationUp:
        case UIImageOrientationUpMirrored:
            break;
    }
    
    switch (self.imageOrientation) {
        case UIImageOrientationUpMirrored:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, self.size.width, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
            
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, self.size.height, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
        case UIImageOrientationUp:
        case UIImageOrientationDown:
        case UIImageOrientationLeft:
        case UIImageOrientationRight:
            break;
    }
    
    // Now we draw the underlying CGImage into a new context, applying the transform
    // calculated above.
    CGContextRef ctx = CGBitmapContextCreate(NULL, self.size.width, self.size.height,
                                             CGImageGetBitsPerComponent(self.CGImage),
                                             CGImageGetBitsPerComponent(self.CGImage)*self.size.width,
                                             CGImageGetColorSpace(self.CGImage),
                                             CGImageGetBitmapInfo(self.CGImage));
    CGContextConcatCTM(ctx, transform);
    switch (self.imageOrientation) {
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            // Grr...
            CGContextDrawImage(ctx, CGRectMake(0,0,self.size.height,self.size.width), self.CGImage);
            break;
            
        default:
            CGContextDrawImage(ctx, CGRectMake(0,0,self.size.width,self.size.height), self.CGImage);
            break;
    }
    
    // And now we just create a new UIImage from the drawing context
    CGImageRef cgimg = CGBitmapContextCreateImage(ctx);
    UIImage *img = [UIImage imageWithCGImage:cgimg];
    CGContextRelease(ctx);
    CGImageRelease(cgimg);
    return img;
}
@end
@implementation customView
@synthesize glView;
@synthesize imgView;
@synthesize re_img_view;
@synthesize motionManager;
@synthesize console1;
@synthesize console2;
@synthesize console3;
@synthesize console4;


- (void)setupCaptureSession
{
    NSError *error = nil;
    AVCaptureSession *session = [[AVCaptureSession alloc] init];
    session.sessionPreset = AVCaptureSessionPreset640x480;
    NSArray *devices = [AVCaptureDevice devices];
    AVCaptureDevice *myDevice;
    for (AVCaptureDevice *device in devices) {
        if ([device hasMediaType:AVMediaTypeVideo]) {
            if ([device position] == AVCaptureDevicePositionBack) {
                myDevice =device;
            }
        }
    }
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:myDevice error:&error];
    if (!input){
        NSLog(@"Device input wrong!!");
    }
    if ([session canAddInput:input]) {
        [session addInput:input];
    }else {
        NSLog(@"add device wrong!!!");
    }
    AVCapturePhotoOutput *output = [[AVCapturePhotoOutput alloc] init];
    photo_output =output;
    
    if ([session canAddOutput:output]) {
        [session addOutput:output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
//    for (AVCaptureConnection *connection in [output connections])
//    {
//        if(connection.supportsVideoOrientation){
//            [connection setVideoOrientation:AVCaptureVideoOrientationLandscapeRight];
//        }
//    }
    AVCaptureVideoPreviewLayer *previewLayer = nil;
    previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:session];
    //[previewLayer setVideoGravity:AVLayerVideoGravityResizeAspect];
    CGRect layerRect = [[imgView layer] bounds];
    [previewLayer setBounds:layerRect];
    previewLayer.orientation = AVCaptureVideoOrientationLandscapeRight;
    [previewLayer setPosition:CGPointMake(CGRectGetMidX(layerRect),CGRectGetMidY(layerRect))];
    [[imgView layer] addSublayer:previewLayer];
    [session startRunning];

}

- (void)captureOutput:(AVCapturePhotoOutput *)captureOutput
didFinishProcessingPhotoSampleBuffer:(CMSampleBufferRef)photoSampleBuffer
previewPhotoSampleBuffer:(CMSampleBufferRef)previewPhotoSampleBuffer
     resolvedSettings:(AVCaptureResolvedPhotoSettings *)resolvedSettings
      bracketSettings:(AVCaptureBracketedStillImageSettings *)bracketSettings
                error:(NSError *)error{
    if (photoSampleBuffer) {
//        NSData *data = [AVCapturePhotoOutput JPEGPhotoDataRepresentationForJPEGSampleBuffer:photoSampleBuffer previewPhotoSampleBuffer:previewPhotoSampleBuffer];
        //UIImage *image = [UIImasucc in extract boardge imageWithData:data];
        UIImage *image = [self imageFromSampleBuffer:photoSampleBuffer];
        img_tool myImg_tool;
        cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
        std::vector<cv::Point2f> img_points;
        img_cv = myImg_tool.cal_corners(img_cv, cv::Size(7,6), img_points);
        if (img_points.size()>0){
            cali_imagePoints.push_back(img_points);
            console1.text = [NSString stringWithFormat:@"img count: %d", cali_imagePoints.size()];
        }
        UIImage* img_re = [mm_Try UIImageFromCVMat:img_cv];
        dispatch_async(dispatch_get_main_queue(), ^{
            [re_img_view setImage:img_re];
        });
    }
}

// Create a UIImage from sample buffer data
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    // Get a CMSampleBuffer's Core Video image buffer for the media data
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    // Lock the base address of the pixel buffer
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    
    // Get the number of bytes per row for the pixel buffer
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    
    // Get the number of bytes per row for the pixel buffer
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    // Get the pixel buffer width and height
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    
    // Create a device-dependent RGB color space
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    // Create a bitmap graphics context with the sample buffer data
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    // Create a Quartz image from the pixel data in the bitmap graphics context
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    // Unlock the pixel buffer
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    
    // Free up the context and color space
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    
    // Create an image object from the Quartz image
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    //UIImage *image = [UIImage imageWithCGImage:quartzImage scale:1.0f orientation:UIImageOrientationRight];
    // Release the Quartz image
    CGImageRelease(quartzImage);
    
    return image;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [glView init_me];
    need_record = false;
    re_img_view.contentMode = UIViewContentModeScaleAspectFit;
    imgView.contentMode = UIViewContentModeScaleAspectFit;
    [self setupCaptureSession];
    photo_cam_off = false;
    first_frame = true;
    cal_translation = false;
    motionManager.deviceMotionUpdateInterval =0.03;
    if (motionManager.deviceMotionAvailable){
        [motionManager
         startDeviceMotionUpdatesToQueue:
         [[NSOperationQueue alloc] init]
         withHandler:
         ^(CMDeviceMotion *data, NSError *error){
             
             cv::Point3f angle;
             cv::Point3f acce;
             std::vector<cv::Mat> allkfs;
             angle.z = data.attitude.roll;
             angle.y = data.attitude.yaw;
             angle.x = data.attitude.pitch;
             acce.x =data.userAcceleration.x;
             acce.y =data.userAcceleration.y;
             acce.z =data.userAcceleration.z;
             cv::Point3f rotRate;
             rotRate.x = data.rotationRate.x;
             rotRate.y = data.rotationRate.y;
             rotRate.z = data.rotationRate.z;
             if (first_frame){
                 long long milliseconds = (long long)([[NSDate date] timeIntervalSince1970] * 1000.0);
                 last_time = milliseconds;
                 first_frame= false;
                 real_acce =cv::Point3f(0,0,0);
                 cur_v=cv::Point3f(0,0,0);
                 cur_posi=cv::Point3f(0,0,0);
                 last_acce = acce;
                 return;
             }
             
             long long milliseconds = (long long)([[NSDate date] timeIntervalSince1970] * 1000.0);
             float delt_t = (milliseconds - last_time)/1000.0;
             last_time = milliseconds;
             cv::Point3f delt_acce = (acce - last_acce)*9.8;
             real_acce = real_acce +delt_acce;
             cur_v = cur_v +real_acce*delt_t;
             cur_posi= cur_posi+cur_v*delt_t +0.5*delt_t*delt_t*real_acce;
             if(need_record==true){
                 atitudes.push_back(angle);
                 acces.push_back(acce);
                 rotRates.push_back(rotRate);
             }
             
             cv::Mat rot = [glView getRotMat:angle.x yaw:angle.y roll:angle.z];
             cv::Mat pose = cv::Mat::eye(4,4,CV_32FC1);
             rot.copyTo(pose.rowRange(0, 3).colRange(0, 3));
             if(cal_translation){
                 pose.at<float>(0,3) =cur_posi.x;
                 pose.at<float>(1,3) =cur_posi.y;
                 pose.at<float>(2,3) =cur_posi.z;
             }
             allkfs.push_back(pose);
             [glView SetKF:allkfs];
             //std::cout<<yaw<<std::endl;
             if(round(render_count/10)*10.0==(float)render_count){
                 dispatch_async(dispatch_get_main_queue(), ^{
                     [glView render];
                 });
                 render_count=0;
             }
             
         }];
    }
}

- (IBAction)start_video:(id)sender {
//    NSArray<NSNumber *> *p_type= photo_output.availablePhotoPixelFormatTypes;
//    for(NSNumber* my_type in p_type){
//        OSType code = [my_type intValue];
//        NSLog(@"%s", FourCC2Str(code));
//        //NSLog(@"%@", @(FourCC2Str(code)));
//    }
    if([motionManager isDeviceMotionActive]){
        [motionManager stopDeviceMotionUpdates];
    }
    AVCapturePhotoSettings *photo_setting;
    NSString* key = (NSString*)kCVPixelBufferPixelFormatTypeKey;
    NSNumber* value = [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA];
    NSDictionary* outputSettings = [NSDictionary dictionaryWithObject:value forKey:key];
    photo_setting = [AVCapturePhotoSettings photoSettingsWithFormat:outputSettings];
    [photo_output capturePhotoWithSettings:photo_setting delegate:self];
}

- (IBAction)start_re:(id)sender {
    need_record=true;
}

- (IBAction)end_re:(id)sender {
    need_record=false;
    std::stringstream ss;
    cv::Point3f rot_sum(0,0,0);
    cv::Point3f acces_sum(0,0,0);
    for(int i=0;i<rotRates.size();i++){
        rot_sum=rotRates[i]+rot_sum;
        acces_sum=acces[i]+acces_sum;
        ss<<rotRates[i].x<<","<<rotRates[i].y<<","<<rotRates[i].z<<","<<acces[i].x<<","<<acces[i].y<<","<<acces[i].z<<std::endl;
    }
    rot_sum.x=rot_sum.x/rotRates.size();
    rot_sum.y=rot_sum.y/rotRates.size();
    rot_sum.z=rot_sum.z/rotRates.size();
    acces_sum.x=acces_sum.x/acces.size();
    acces_sum.y=acces_sum.y/acces.size();
    acces_sum.z=acces_sum.z/acces.size();
    ss<<"avi: "<<std::endl;
    ss<<rot_sum.x<<","<<rot_sum.y<<","<<rot_sum.z<<","<<acces_sum.x<<","<<acces_sum.y<<","<<acces_sum.z<<std::endl;
    send_txt(ss.str());
    atitudes.clear();
    acces.clear();
    rotRates.clear();
}
- (IBAction)resetIMU:(id)sender {
    first_frame = true;
    cal_translation = true;
}
- (IBAction)cali_all:(id)sender {
    Settings s;
    s.init();
    s.boardSize=cv::Size(7,6);
    s.aspectRatio =640/(float)480;
    s.img_size = cv::Size(640,480);
    s.squareSize = 0.032;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> tvecs;
    std::vector<float> reprojErrs;
    std::vector<cv::Mat> rvecs;
    double totalAvgErr;
    std::vector<cv::Point3f> mps;
    img_tool::runCalibration(s, cameraMatrix, distCoeffs, cali_imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr, mps);
    std::vector<MapPointChamo> mps_vis;
    for (int i=0; i<mps.size();i++){
        MapPointChamo mp;
        mp.posi =mps[i];
        mps_vis.push_back(mp);
    }
    [glView SetMapPoints:mps_vis];
    std::vector<cv::Mat> kfs;
    for (int i=0;i<rvecs.size();i++){
        cv::Mat pose = cv::Mat::eye(4,4,CV_32FC1);
        cv::Mat rot_f;
        cv::Mat trans_f;
        cv::Rodrigues(rvecs[i], rot_f);
        rot_f.convertTo(rot_f, CV_32F);
        tvecs[i].convertTo(trans_f, CV_32F);
        rot_f.copyTo(pose.rowRange(0, 3).colRange(0, 3));
        trans_f.copyTo(pose.rowRange(0,3).col(3));
        std::cout<<pose<<std::endl;
        kfs.push_back(pose);

    }
    [glView SetKF:kfs];
    [glView render];
    std::string contnent;
    std::stringstream ss;
    ss<<"cameraMatrix: "<<std::endl;
    ss<<cameraMatrix<<std::endl;
    ss<<"distCoeffs: "<<std::endl;
    ss<<distCoeffs<<std::endl;
    ss<<"totalAvgErr: "<<totalAvgErr<<std::endl;
    contnent =ss.str();
    send_txt(contnent);
}
@end
