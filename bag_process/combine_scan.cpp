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

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


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

void tranforam_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr new_pt_cloud,Eigen::Matrix4f pose){
    for (int j=0; j<pt_cloud->points.size(); j++){
        Eigen::Vector4f p(pt_cloud->points[j].x, pt_cloud->points[j].y, pt_cloud->points[j].z, 1);
        Eigen::Vector4f p1=pose*p;
        pcl::PointXYZ p_pcl(p1.x(), p1.y(), p1.z());
        new_pt_cloud->points.push_back(p_pcl);
    }
}

bool get_nn_pose(std::map<double, Eigen::Matrix4f>& poses, double cur_time ,Eigen::Matrix4f& out_pose){
    std::map<double, Eigen::Matrix4f>::iterator it1 =poses.lower_bound (cur_time); 
    if(it1== poses.end()){
        return false;
    }
    std::map<double, Eigen::Matrix4f>::iterator it2=it1--;
    if(it2== poses.end()){
        return false;
    }
    if(cur_time-it1->first<0 || it2->first-cur_time<0){
        std::cout<<"timeline wrong!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first>0.01 || it2->first-cur_time>0.01){
        std::cout<<"too far!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first >it2->first-cur_time){
        out_pose=it2->second;
    }else{
        out_pose=it1->second;
    }
    return true;
}

int main(int argc, char **argv)
{
    int num_package_in_scan=151;
    
    std::string bag_name =argv[1];
    std::string traj_file =argv[2];
    
    std::string input_bag=bag_name;
    std::string output_bag=input_bag;
    
    size_t pos = bag_name.find(".bag");
    if (std::string::npos == pos) {
        std::cout << "The bag file name is not end with '*.bag' " << std::endl;
        return 1;
    }
    output_bag.insert(pos, "_fixed");

    std::cout << "Input bag:  " << input_bag << std::endl;
    std::cout << "Output bag: " << output_bag << std::endl;

    rosbag::Bag in_bag {input_bag, rosbag::bagmode::Read};
    rosbag::Bag out_bag {output_bag, rosbag::bagmode::Write};
    std::clock_t time = std::clock();
    
    std::map<double, Eigen::Matrix4f> pose_list;
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

        Eigen::Vector3f trans=Eigen::Vector3f(x,y,z);
        Eigen::Quaternionf qua=Eigen::Quaternionf(qw, qx, qy, qz);
        Eigen::Matrix3f R = qua.toRotationMatrix();
        Eigen::Matrix4f tran_mat= Eigen::Matrix4f::Identity();
        tran_mat.block<3,3>(0,0)=R;
        tran_mat.block<3,1>(0,3)=trans;
        pose_list[time_stamp]=tran_mat;
    }
    
    int lidar_package_count=0;
    Eigen::Matrix4f Twr;
    ros::Time ref_time;
    double ref_header_time=-1;
    std::cout<<"start process bag"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (rosbag::MessageInstance m : rosbag::View{in_bag}) {
        if (m.getTopic() == "velodyne_points") {
            sensor_msgs::PointCloud2Ptr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            double lidar_time=lidar_msg->header.stamp.toSec();
            Eigen::Matrix4f Twl;
            bool re = get_nn_pose(pose_list, lidar_time, Twl);
            if(re==false){
                continue;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg (*lidar_msg, *pt_cloud);
            
            if (lidar_package_count%num_package_in_scan==0){
                std::cout<<"package count: "<<lidar_package_count<<std::endl;
                if(ref_header_time>0){ //not first package
                    sensor_msgs::PointCloud2 new_cloud; 
                    pcl::toROSMsg(*combine_cloud, new_cloud);
                    new_cloud.header.frame_id = "world";
                    new_cloud.header.stamp=ros::Time(ref_header_time);
                    out_bag.write("velodyne_points", ref_time, new_cloud);
                    tf::Transform transform;
                    Eigen::Vector3f posi=Twr.block<3,1>(0,3);
                    transform.setOrigin( tf::Vector3(posi.x(), posi.y(), posi.z()) );
                    Eigen::Quaternionf qua(Twr.block<3,3>(0,0));
                    tf::Quaternion q(qua.x(), qua.y(), qua.z(), qua.w());
                    transform.setRotation(q);
                    geometry_msgs::TransformStamped msg;
                    tf::transformStampedTFToMsg(tf::StampedTransform(transform, ros::Time(ref_header_time), "world","velodyne"),msg);
                    tf::tfMessage tfmsg;
                    tfmsg.transforms.push_back(msg);
                    out_bag.write("/tf", ref_time, tfmsg);
                }
                ref_time= m.getTime();
                ref_header_time=lidar_time;
                Twr=Twl;
                combine_cloud=pt_cloud;
            }else{
                Eigen::Matrix4f Trl=Twr*Twl.inverse();
                pcl::PointCloud<pcl::PointXYZ>::Ptr new_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                tranforam_pc(pt_cloud, new_pt_cloud,Trl);
                *combine_cloud+=*new_pt_cloud;
            }
            lidar_package_count++;
        }
    }
    

    in_bag.close();
    out_bag.close();

    std::cout << "Process time: " << (std::clock() - time) / 1000000 << "s" << std::endl;
    std::cout << "Done" << std::endl;
};
