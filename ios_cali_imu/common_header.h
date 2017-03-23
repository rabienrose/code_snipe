//
//  common_header.h
//  opencv_ios_try
//
//  Created by test on 10/1/16.
//  Copyright © 2016 test. All rights reserved.
//

#ifndef common_header_h
#define common_header_h
#import<Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#include <opencv2/opencv.hpp>

struct MapPointChamo{
    cv::Point3f posi;
};

@interface mm_Try : NSObject
+(int) add:(int)a b:(int)b;
+(cv::Mat)cvMatFromUIImage:(UIImage *)image;
+(UIImage *) convertImag: (UIImage *)image;
+(cv::Mat)cvMatGrayFromUIImage:(UIImage *)image;
+(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat;
@end
void send_txt(std::string contnent);
void send_file(std::string file_name);
void send_file_default_folder(std::string file_name);
bool save_txt_file(std::string contnent, std::string file_name);
#endif /* common_header_h */
