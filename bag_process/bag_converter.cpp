#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <sys/types.h> 

namespace enc = sensor_msgs::image_encodings;
int main(int argc, char **argv){

    //this program will convert the specified topic to mono image, the converted image will be stored with a new topic name
    std::string bag_addr=argv[1]; //full address of bag file
    std::string in_topic=argv[2]; //the topic you want to convert
    std::string out_topic=argv[3]; //the name of topic to store the converted data
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);      
    std::vector<std::string> topics;
    topics.push_back(in_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    std::vector<sensor_msgs::Image> imgs; 
    double first_img_time=0;
    int img_count=0;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg_jpg = m.instantiate<sensor_msgs::CompressedImage>();
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImagePtr cv_ptr;
        
        if(simg_jpg!=NULL || simg!=NULL){
            std_msgs::Header header;
            cv::Mat img;
            std::cout<<m.getTopic()<<" : ";
            std::cout<<std::setprecision (14)<<m.getTime().toSec()<<" : ";
            if(simg_jpg!=NULL){
                std::cout<<simg_jpg->header.seq<<" : ";
                std::cout<<simg_jpg->header.stamp.toSec()<<std::endl;
                cv_ptr = cv_bridge::toCvCopy(simg_jpg, "mono8");
                img=cv_ptr->image;
                header=simg_jpg->header;
            }
            if(simg!=NULL){
                std::cout<<simg->header.seq<<" : ";
                std::cout<<simg->header.stamp.toSec()<<" : ";
                try
                {
                    if (enc::isMono(simg->encoding) || enc::isColor(simg->encoding))
                    {
                        cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                        img=cv_ptr->image;
                    }
                    else if (enc::isBayer(simg->encoding))
                    {
                        const cv::Mat bayer(simg->height, simg->width, CV_MAKETYPE(CV_8U, 1), const_cast<uint8_t*>(&simg->data[0]), simg->step);
                        int code = -1;
                        if (simg->encoding == enc::BAYER_RGGB8 || simg->encoding == enc::BAYER_RGGB16){
                            code = cv::COLOR_BayerBG2GRAY;
                            std::cout<<"BAYER_RGGB8"<<std::endl;
                        }
                        else if (simg->encoding == enc::BAYER_BGGR8 ||simg->encoding == enc::BAYER_BGGR16){
                            code = cv::COLOR_BayerRG2GRAY;
                            std::cout<<"BAYER_BGGR8"<<std::endl;
                        }
                        else if (simg->encoding == enc::BAYER_GBRG8 ||simg->encoding == enc::BAYER_GBRG16){
                            code = cv::COLOR_BayerGR2GRAY;
                            std::cout<<"BAYER_GBRG8"<<std::endl;
                        }
                        else if (simg->encoding == enc::BAYER_GRBG8 ||simg->encoding == enc::BAYER_GRBG16){
                            code = cv::COLOR_BayerGB2GRAY;
                            std::cout<<"BAYER_GRBG8"<<std::endl;
                        }
                        cv::cvtColor(bayer, img, code);
                    }
                    header=simg->header;
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return 0;
                }
            }
            sensor_msgs::Image new_img_msg;
            new_img_msg.height=img.rows;
            new_img_msg.width=img.cols;
            new_img_msg.encoding="mono8";
            new_img_msg.is_bigendian=false;
            new_img_msg.step=img.cols;
            std::vector<uchar> array;
            if (img.isContinuous()) {
                array.assign(img.datastart, img.dataend);
            } else {
                for (int i = 0; i < img.rows; ++i) {
                    array.insert(array.end(), img.ptr<uchar>(i), img.ptr<uchar>(i)+img.cols);
                }
            }
            new_img_msg.data=array;
            new_img_msg.header=header;
            imgs.push_back(new_img_msg);    
            img_count++;    
        }
    }
    bag.close();
    
    rosbag::Bag bag_save;
    bag_save.open(bag_addr,rosbag::bagmode::Append);  
    for(int i=0; i<imgs.size();i++){
        bag_save.write(out_topic, imgs[i].header.stamp, imgs[i]);
    }
    bag_save.close();
    

    
    return 0;
};
