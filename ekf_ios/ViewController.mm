#import "ViewController.h"
#import <CoreMotion/CoreMotion.h>
#include <Eigen/Eigen>

@interface ViewController ()

@end

@implementation ViewController
@synthesize glViewer;
@synthesize camViewer;
@synthesize reViewer;
CMMotionManager *motionManager;

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
    AVCaptureVideoDataOutput *output = [[AVCaptureVideoDataOutput alloc] init];
    dispatch_queue_t queue = dispatch_queue_create("myQueue", NULL);
    [output setSampleBufferDelegate:self queue:queue];
    output.videoSettings =[NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA] forKey:(id)kCVPixelBufferPixelFormatTypeKey];
    if ([session canAddOutput:output]) {
        [session addOutput:output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    AVCaptureVideoPreviewLayer *previewLayer = nil;
    previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:session];
    CGRect layerRect = [[camViewer layer] bounds];
    [previewLayer setBounds:layerRect];
    previewLayer.orientation = AVCaptureVideoOrientationLandscapeRight;
    [previewLayer setPosition:CGPointMake(CGRectGetMidX(layerRect),CGRectGetMidY(layerRect))];
    [[camViewer layer] addSublayer:previewLayer];
    [session startRunning];
}

// Create a UIImage from sample buffer data
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection{
    double current_time = [[NSDate date] timeIntervalSince1970]-start_time;
    double temp_time = current_time -last_time;
    if (sampleBuffer && running && temp_time>0.3) {
        last_time = current_time;
        UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
        cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
        cv::Mat img_cv_3c;
        cv::cvtColor(img_cv, img_cv_3c, CV_BGRA2BGR);
        dispatch_async(queue, ^{
            vis_count++;
            if(vis_count%1==0){
                vis_count= 0;
                Eigen::Vector3d posi_eigen = tonav_->getCurrentPosition();
                Quaternion ori_q = tonav_->getCurrentOrientation();
                cv::Mat posi_m(3,1, CV_32F);
                posi_m.at<float>(0,0) =posi_eigen[0];
                posi_m.at<float>(2,0) =posi_eigen[1];
                posi_m.at<float>(1,0) =posi_eigen[2];
                cv::Mat rot_m(3,3,CV_32FC1);
                Eigen::Matrix3d ori_eigen = ori_q.toRotationMatrix();
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        rot_m.at<float>(i,j) =ori_eigen(i,j);
                    }
                }
                posi_m = posi_m- pose_offset;
                cv::Mat pose = [glViewer GetPoseFromRT:rot_m posi:posi_m];
                std::vector<cv::Mat> poses;
                poses.push_back(pose);
                [glViewer SetKF:poses];
                cv::Mat img_re_cv = tonav_->getCurrentImage().clone();
                if (!img_re_cv.empty()){
                    cv::cvtColor(img_re_cv, img_re_cv, CV_BGR2BGRA);
                    UIImage* img_re = [mm_Try UIImageFromCVMat:img_re_cv];
                    dispatch_async(dispatch_get_main_queue(), ^{
                        [reViewer setImage:img_re];
                        [glViewer render];
                    });
                }
            }
            //std::cout<<"updateImage "<<current_time<<std::endl;
            tonav_->updateImage(current_time, img_cv_3c);
        });
    }
}

- (void)viewDidLoad {
    [super viewDidLoad];
    running = false;
    is_initialized =false;
    vis_count=0;
    [glViewer init_me];
    motionManager = [[CMMotionManager alloc] init];
    motionManager.gyroUpdateInterval =0.1;
    motionManager.accelerometerUpdateInterval =0.1;
    [motionManager startDeviceMotionUpdates];
    queue = dispatch_queue_create("my_queue", NULL);
    pose_offset= cv::Mat::zeros(3, 1, CV_32FC1);
    [self setupCaptureSession];
    if(motionManager.deviceMotionActive){
        [motionManager startDeviceMotionUpdates];
    }
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

Eigen::Quaterniond euler2Quaternion( const double roll, const double pitch, const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

- (IBAction)start_algo:(id)sender {
    running= !running;
    if(running==true){
        start_time = [[NSDate date] timeIntervalSince1970];
        last_time =start_time;
        NSString* cali_path = [[NSBundle mainBundle] pathForResource:@"calibration" ofType:@"yaml"];
        std::string cali_path_std([cali_path UTF8String]);
        std::shared_ptr<Calibration> calibration_ = Calibration::fromPath(cali_path_std);
        calibration_->setCameraOpticalCenter(calibration_->getCameraOpticalCenter() );
        calibration_->setCameraFocalPoint(calibration_->getCameraFocalPoint());
        calibration_->setCameraRadialDistortionParams(Eigen::Vector3d::Zero());
        calibration_->setCameraTangentialDistortionParams(Eigen::Vector2d::Zero());
        calibration_->setCameraDelayTime(0.0);
        calibration_->setCameraReadoutTime(0.0);
        Eigen::Matrix3d R_B_C_;
        R_B_C_.setIdentity();
        calibration_->setBodyToCameraRotation(Quaternion::fromRotationMatrix(R_B_C_.transpose()));
        std::shared_ptr<FeatureTracker> feature_tracker = std::make_shared<FeatureTracker>(calibration_->getNumberOfFeaturesToExtract());
        tonav_.reset(new Tonav(calibration_, Eigen::Vector3d(0,0,0), feature_tracker));
        Eigen::Vector3d velocity;
        velocity << 0, 0, 0;
        tonav_->initializer()->setPosition(Eigen::Vector3d(0,0,0));
        tonav_->initializer()->setVelocity(velocity);
        double w = motionManager.deviceMotion.attitude.quaternion.w();
        double x = motionManager.deviceMotion.attitude.quaternion.x();
        double y = motionManager.deviceMotion.attitude.quaternion.y();
        double z = motionManager.deviceMotion.attitude.quaternion.z();
        tonav_->initializer()->setOrientation(Quaternion(w, x, y,z));
        last_time =0;
        std::vector<cv::Mat> none_vec_mat;
        [glViewer SetKF:none_vec_mat];
        std::vector<MapPointChamo> none_vec_mp;
        [glViewer SetMapPoints:none_vec_mp];
        if(motionManager.gyroAvailable){
            [motionManager startGyroUpdatesToQueue: [[NSOperationQueue alloc] init]
             withHandler: ^(CMGyroData *data, NSError *error){
                 if(running){
                     double current_time = [[NSDate date] timeIntervalSince1970]-start_time;
                     Eigen::Vector3d gyro;
                     gyro[1]=data.rotationRate.x;
                     gyro[0]=data.rotationRate.y;
                     gyro[2]=data.rotationRate.z;
                     dispatch_async(queue, ^{
                         //std::cout<<"updateIMU "<<current_time<<std::endl;
                         tonav_->updateRotationRate(current_time, gyro);
                     });
                 }
             }];
        }
        if(motionManager.accelerometerAvailable){
            [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMAccelerometerData *data, NSError *error)
            {
                if(running){
                    double current_time = [[NSDate date] timeIntervalSince1970]-start_time;
                    Eigen::Vector3d accel;
                    accel[0]=data.acceleration.x*9.81;
                    accel[1]=data.acceleration.y*9.81;
                    accel[2]=data.acceleration.z*9.81;
                    std::cout<<accel.transpose()<<std::endl;
                    dispatch_async(queue, ^{
                        tonav_->updateAcceleration(current_time, accel);
                    });
                }
            }];
         }
    }
}

- (IBAction)set_ori:(id)sender {
    Eigen::Vector3d posi_eigen = tonav_->getCurrentPosition();
    pose_offset.at<float>(0,0) =posi_eigen[0];
    pose_offset.at<float>(2,0) =posi_eigen[1];
    pose_offset.at<float>(1,0) =posi_eigen[2];
}
@end
