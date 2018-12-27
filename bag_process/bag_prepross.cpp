#include <iostream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <rosgraph_msgs/Log.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    if (2 != argc) {
    }

    std::clock_t time = std::clock();

    std::string in_bag_name {argv[1]};
    std::string out_bag_name {argv[1]};
    size_t pos = out_bag_name.find(".bag");
    if (std::string::npos == pos) {
        std::cout << "The bag file name is not end with '*.bag' " << std::endl;
        return 1;
    }
    out_bag_name.insert(pos, "_fixed");

    std::cout << "Input bag:  " << in_bag_name << std::endl;
    std::cout << "Output bag: " << out_bag_name << std::endl;

    rosbag::Bag in_bag {in_bag_name, rosbag::bagmode::Read};
    rosbag::Bag out_bag {out_bag_name, rosbag::bagmode::Write};

    std::cout << "Starting fixing" << std::endl;
    
    bool savejpg=false;
    bool resize=false;
    bool savecolor=false;

    ros::Time last_time;
    
    int img_count=0;

    for (rosbag::MessageInstance m : rosbag::View{in_bag}) {
        if (m.getTopic() == "imu" || m.getTopic() == "/imu/raw_data") {
            //sensor_msgs::ImuPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            //imu_msg->header.stamp=imu_msg->header.stamp-ros::Duration(0.5);
            out_bag.write("/imu/raw_data", m.getTime(), m.instantiate<sensor_msgs::Imu>());
            continue;
        }
        
        if (m.getTopic() == "velodyne_points" || m.getTopic() == "/velodyne_points") {
            sensor_msgs::PointCloud2Ptr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            lidar_msg->header.frame_id="velodyne";
            out_bag.write("velodyne_points", m.getTime(), lidar_msg, m.getConnectionHeader());
            continue;
        }

        if (m.getTopic() == "img" || m.getTopic() == "camera/right/image_raw") {
            if(savejpg){
                cv_bridge::CvImageConstPtr cv_ptr;
                //sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
                sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
                try {
                    if(savecolor){
                        cv_ptr = cv_bridge::toCvCopy(simg, sensor_msgs::image_encodings::BGR8);
                    }else{
                        cv_ptr = cv_bridge::toCvCopy(simg, sensor_msgs::image_encodings::MONO8);
                    }
                    
                } catch (const cv_bridge::Exception& e) {
                    std::cout << "cv_bridge exception: " << std::endl;
                }
//                 cv::Mat resized_img;
//                 if(resize){
//                     cv::resize(cv_ptr->image, resized_img, cv::Size(cv_ptr->image.cols/2, cv_ptr->image.rows/2));
//                 }else{
//                     resized_img=cv_ptr->image;
//                 }
                
//                 sensor_msgs::CompressedImage img_ros_img;
//                 std::vector<unsigned char> binaryBuffer_;
//                 cv::imencode(".jpg", resized_img, binaryBuffer_);
//                 img_ros_img.data=binaryBuffer_;
//                 img_ros_img.header=simg->header;
//                 img_ros_img.format="jpeg";
//                 out_bag.write("img", m.getTime(), img_ros_img);
                sensor_msgs::ImagePtr img_ros_img= cv_ptr->toImageMsg();
                //out_bag.write("img", m.getTime(), img_ros_img);
                img_count++;
                if(img_count>1000){
                    break;
                }
            }else{
                //sensor_msgs::ImagePtr img_msg = m.instantiate<sensor_msgs::Image>();
                out_bag.write("camera/right/image_raw", m.getTime(), m.instantiate<sensor_msgs::Image>());
                img_count++;
                if(img_count>100){
                    //break;
                }
            }
            
            continue;
        }

    }
    in_bag.close();
    out_bag.close();

    std::cout << "Process time: " << (std::clock() - time) / 1000000 << "s" << std::endl;
    std::cout << "Done" << std::endl;
};
