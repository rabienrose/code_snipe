#include <iostream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf/tfMessage.h>
#include <rosgraph_msgs/Log.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>


std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

int main(int argc, char **argv)
{

    std::clock_t time = std::clock();
    std::string out_bag_name =argv[1];
    std::string traj_file =argv[2];

    std::cout << "Output bag: " << out_bag_name << std::endl;

    rosbag::Bag out_bag {out_bag_name, rosbag::bagmode::Append};
    
    
    std::ifstream infile(traj_file);
    std::string line;
    while(true){
        std::getline(infile, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, "  ");
        double time_stamp;
        time_stamp=atof(splited[0].c_str());
        double x=atof(splited[1].c_str());
        double y=atof(splited[2].c_str());
        double z=atof(splited[3].c_str());
        double qw=atof(splited[4].c_str());
        double qx=atof(splited[5].c_str());
        double qy=atof(splited[6].c_str());
        double qz=atof(splited[7].c_str());
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q(qx, qy, qz, qw);
        transform.setRotation(q);
        geometry_msgs::TransformStamped msg;
        tf::transformStampedTFToMsg(tf::StampedTransform(transform, ros::Time(time_stamp), "world","velodyne"),msg);
        tf::tfMessage tfmsg;
        tfmsg.transforms.push_back(msg);
        out_bag.write("/tf", ros::Time(time_stamp),tfmsg);
    }
    

    out_bag.close();

    std::cout << "Process time: " << (std::clock() - time) / 1000000 << "s" << std::endl;
    std::cout << "Done" << std::endl;
};
