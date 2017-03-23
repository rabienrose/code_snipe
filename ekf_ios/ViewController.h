#import "OpenGLView.h"
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#include "tonav.h"

@interface ViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    std::shared_ptr<Tonav> tonav_;
    bool running;
    bool is_initialized;
    int vis_count;
    double start_time;
    double last_time;
    dispatch_queue_t queue;
    cv::Mat pose_offset;
}
@property (weak, nonatomic) IBOutlet OpenGLView *glViewer;
@property (weak, nonatomic) IBOutlet UIButton *set_ori_btn;
@property (weak, nonatomic) IBOutlet UIImageView *camViewer;
@property (weak, nonatomic) IBOutlet UIButton *start_btn;
- (IBAction)start_algo:(id)sender;
- (IBAction)set_ori:(id)sender;
@property (weak, nonatomic) IBOutlet UIImageView *reViewer;
@end

