#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "chamo_tf");
  ros::NodeHandle node;
  tf2_ros::StaticTransformBroadcaster tfb;
  ros::Rate rate(1);
  int count=0;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "world";
    std::ostringstream ss;
    ss<<"chamo_tf_"<<count; 
    transformStamped.child_frame_id = ss.str().c_str();
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    //transformStamped.header.stamp = ros::Zero (0);
    tfb.sendTransform(transformStamped);
    printf("sending\n");
    rate.sleep();
    uint32_t shape = visualization_msgs::Marker::CUBE;
    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);    
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseArray>("poses", 1);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("clouds", 1);    
    count=0;
    
        while (marker_pub.getNumSubscribers() < 1)
        {
          if (!node.ok())
          {
            return 0;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        std::cout<<"subscri count: "<<marker_pub.getNumSubscribers()<<std::endl;
    while (node.ok()){
        if(count>=1){
            //break;    
        }
        count++;
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markers;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "track";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
//        for (int i = 0; i < 10; ++i) {
//        	geometry_msgs::Point p;
//            p.x = -i;
//            p.y = i;
//            p.z = 0;   
//        	marker.points.push_back(p);
//        }
        marker.lifetime = ros::Duration();
        std::cout<<count<<std::endl;
        markers.markers.push_back(marker);
        marker.id=1;
        marker.pose.position.x = 3;
        markers.markers.push_back(marker);
        marker.pose.position.x = 5;
        marker.id=2;
        markers.markers.push_back(marker);
        marker_pub.publish(markers);
        
        geometry_msgs::PoseArray poses;
        geometry_msgs::Pose pose;
        poses.header.frame_id = "/world";
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        poses.poses.push_back(pose);
        pose.position.x = 1;
        poses.poses.push_back(pose);
        pose.position.y = 2;
        poses.poses.push_back(pose);
        pose_pub.publish(poses);
        
        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id="/world";
	    cloud.points.resize(10);
	    cloud.channels.resize(1);
	    cloud.channels[0].name = "intensities";
	    cloud.channels[0].values.resize(10);
	    for (std::size_t i = 0; i < 10; ++i) {
	        cloud.points[i].x = i;
	        cloud.points[i].y = 0;
	        cloud.points[i].z = 0;
	        cloud.channels[0].values[i] = 1;
	    }
	    cloud_pub.publish(cloud);
        rate.sleep();
    }
  return 0;
};