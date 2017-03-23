#import "common_header.h"
#include <iostream>
#include <fstream>

@implementation mm_Try

+(int) add:(int)a b:(int)b{
    return a+b;
}

+(UIImage *) convertImag: (UIImage *)image
{
    cv::Mat img = [mm_Try cvMatFromUIImage:image];
    cv::Mat greyMat;
    cv::cvtColor(img, greyMat, CV_BGR2GRAY);
    //cv::resize(greyMat, greyMat, cv::Size(320,460));
    UIImage * reImg = [mm_Try UIImageFromCVMat:greyMat];
    return reImg;
}

+(cv::Mat)cvMatFromUIImage: (UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGBitmapByteOrder32Little |
                                                    kCGImageAlphaPremultipliedFirst); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}

+ (cv::Mat)cvMatGrayFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC1); // 8 bits per component, 1 channels
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}

+(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat
{
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize()*cvMat.total()];
    CGColorSpaceRef colorSpace;
    
    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                 //width
                                        cvMat.rows,                                 //height
                                        8,                                          //bits per component
                                        8 * cvMat.elemSize(),                       //bits per pixel
                                        cvMat.step[0],                            //bytesPerRow
                                        colorSpace,                                 //colorspace
                                        kCGImageAlphaPremultipliedFirst|kCGBitmapByteOrder32Little,// bitmap info
                                        provider,                                   //CGDataProviderRef
                                        NULL,                                       //decode
                                        false,                                      //should interpolate
                                        kCGRenderingIntentDefault                   //intent
                                        );
    
    
    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return finalImage;
}

void send_txt(std::string contnent) {
    NSData *data = [@(contnent.c_str()) dataUsingEncoding:NSUTF8StringEncoding];
    NSURL *URL = [NSURL URLWithString:@"http://zili-wang.com/file_upload.php"];
    NSMutableURLRequest* request = [NSMutableURLRequest requestWithURL:URL];
    [request setHTTPMethod:@"POST"];
    NSURLSession *session = [NSURLSession sharedSession];
    NSURLSessionUploadTask *uploadTask = [session uploadTaskWithRequest:request
                                                               fromData:data
                                                      completionHandler:
                                          ^(NSData *data, NSURLResponse *response, NSError *error) {
                                              //NSLog([[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
                                          }];
    [uploadTask resume];
}



void send_file(std::string file_name){
    NSURL *URL = [NSURL URLWithString:@"http://zili-wang.com/file_upload.php"];
    NSURL *URL_file = [NSURL URLWithString:@(file_name.c_str())];
    NSMutableURLRequest* request = [NSMutableURLRequest requestWithURL:URL];
    [request setHTTPMethod:@"POST"];
    NSURLSession *session = [NSURLSession sharedSession];
    NSURLSessionUploadTask *uploadTask = [session uploadTaskWithRequest:request
                                                               fromFile:URL_file
                                                      completionHandler:
                                          ^(NSData *data, NSURLResponse *response, NSError *error) {
                                              //NSLog([[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
                                          }];
    [uploadTask resume];
}

void send_file_default_folder(std::string file_name){
    NSString *documentsPath = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) firstObject];
    std::string file_name_addr = std::string([documentsPath UTF8String]);
    file_name_addr=file_name_addr+"/"+file_name;
    send_file(file_name_addr);
}

bool save_txt_file(std::string contnent, std::string file_name){
    NSString *documentsPath = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) firstObject];
    std::ofstream myfile;
    std::string file_name_addr = std::string([documentsPath UTF8String]);
    file_name_addr=file_name_addr+"/"+file_name;
    myfile.open (file_name_addr);
    if (myfile.is_open()){
        myfile<<file_name;
        myfile.close();
        return true;
    }else{
        return false;
    }
    
}

@end
