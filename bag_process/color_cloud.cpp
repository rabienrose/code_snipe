#include "utility.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

cv::Mat cameraExtrinsicMat_;    //camera laser calibration param
cv::Mat cameraMat_;             //camera internal parameter
cv::Mat distCoeff_;             //camera distortion parameter
cv::Size imageSize_;            //image size
cv::Mat R_;                     //lasr to image rotation matrix
cv::Mat T_;                     //lasr to image translation matrix

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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn, cv::Mat img){

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

    int cloudSize = cloudIn->points.size();
    //cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){
        PointType *pointFrom;
        pcl::PointXYZRGBA pointTo;

        pointFrom = &cloudIn->points[i];
        pointTo.a = 255;
        pointTo.r = 0;
        pointTo.g = 255;
        pointTo.b = 0;

        //transfer point cloud into camera frame to map color
        cv::Mat point(3, 1, CV_64F);
        point.at<double>(0) = pointFrom->z; //loam back to lidar because RT is lidar to camera 
        point.at<double>(1) = pointFrom->x;
        point.at<double>(2) = pointFrom->y;
        point = R_ * point  + T_;
    
        if(point.at<double>(2)>0.2 && point.at<double>(2)<10)
        {
            double tmpx = point.at<double>(0) / point.at<double>(2);
            double tmpy = point.at<double>(1)/point.at<double>(2);
            cv::Point2d imagepoint;
            double r2 = tmpx * tmpx + tmpy * tmpy;
            double tmpdist = 1 + distCoeff_.at<double>(0) * r2
               + distCoeff_.at<double>(1) * r2 * r2
               + distCoeff_.at<double>(4) * r2 * r2 * r2;
            imagepoint.x = tmpx * tmpdist
               + 2 * distCoeff_.at<double>(2) * tmpx * tmpy
               + distCoeff_.at<double>(3) * (r2 + 2 * tmpx * tmpx);
            imagepoint.y = tmpy * tmpdist
               + distCoeff_.at<double>(2) * (r2 + 2 * tmpy * tmpy)
               + 2 * distCoeff_.at<double>(3) * tmpx * tmpy;

            //imagepoint.x = tmpx;
            //imagepoint.y = tmpy;

            imagepoint.x = cameraMat_.at<double>(0,0) * imagepoint.x + cameraMat_.at<double>(0,2);
            imagepoint.y = cameraMat_.at<double>(1,1) * imagepoint.y + cameraMat_.at<double>(1,2);
            int px = int(imagepoint.x + 0.5);
            int py = int(imagepoint.y + 0.5);
            if(0 <= px && px < imageSize_.width && 400 <= py && py < imageSize_.height)
            {
                //int pid = py * w + px;
                pointTo.r = img.at<cv::Vec3b>(py,px)[0];
                pointTo.g = img.at<cv::Vec3b>(py,px)[1];
                pointTo.b = img.at<cv::Vec3b>(py,px)[2];
                pointTo.a = 255;
            }//if cloud poind belongs to image  
            else{
                continue;
            }
        }//if z>0   
        else{
            continue;
        }

        //transfer local to global 
        
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        //pointTo.intensity = pointFrom->intensity;

        cloudOut->points.push_back(pointTo);
    }
    return cloudOut;
}
int readCalibPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>cameraMat_;
    fs[DISTCOEFF]>>distCoeff_;
    fs[CAMERAEXTRINSICMAT]>>cameraExtrinsicMat_;
    fs[IMAGESIZE]>>imageSize_;

    R_ = cameraExtrinsicMat_(cv::Rect(0,0,3,3));
    T_ = cameraExtrinsicMat_(cv::Rect(3,0,1,3));
    cout<<"cameraMat:"<<cameraMat_<<endl;
    cout<<"distCoeff:"<<distCoeff_<<endl;
    cout<<"cameraExtrinsicMat:"<<cameraExtrinsicMat_<<endl;
    cout<<"imageSize:"<<imageSize_<<endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "merge_cloud");
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    
    std::string pathBag="/media/albert/新加卷/TEST_DATA/jili_data/ecarx_2018_10_31_bs_dark_fixed.bag";
    std::string traj_path="/mnt/nfs/hdmap/lidar_slam/ecarx_2018_10_31_bs_dark_result/traj.txt";
    std::string point_topic="velodyne_points";
    std::string image_topic="camera/left/image_raw";
    std::string root_save_folder="/media/albert/新加卷/TEST_DATA/jili_data/";
    
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    std::ifstream infile(traj_path);
    
    //read camera intrinsic and extrinsic parameters
    readCalibPara("/media/albert/新加卷/TEST_DATA/jili_data/calib_camera_1.yml");
    
    //read pose
    while (true)
    {
        std::string transform_str;
        std::getline(infile, transform_str);
        if (transform_str==""){
            break;
        }
        std::vector<std::string> splited= split(transform_str, " ");
        PointTypePose pose_t;
        pose_t.time=std::stod(splited[1]);
        pose_t.x=std::stod(splited[2]);
        pose_t.y=std::stod(splited[3]);
        pose_t.z=std::stod(splited[4]);
        pose_t.roll=std::stod(splited[5]);
        pose_t.pitch=std::stod(splited[6]);
        pose_t.yaw=std::stod(splited[7]);
        cloudKeyPoses6D->push_back(pose_t);
    }
    
    vector<pcl::PointCloud<PointType>::Ptr> all_cloudpoint;
    vector<cv::Mat> all_image;
    std::vector<double> cp_timestamps;
    std::vector<double> img_timestamps;
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);
    rosbag::View view(bag);
    int read_count=0;
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::PointCloud2::ConstPtr s1 = m.instantiate<sensor_msgs::PointCloud2>();
    
        if (s1 != NULL && m.getTopic() == point_topic) {
            if (read_count!=0){
                read_count++;
                continue;
            }else{
                read_count=0;
            }
            
            pcl::fromROSMsg(*s1, *laserCloudIn);
            pcl::PointCloud<PointType>::Ptr temp_pointcloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr out_pointcloud(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudIn, *temp_pointcloud);
            PointType point;
            int cloudSize = temp_pointcloud->points.size();
            for (int i = 0; i < cloudSize; ++i)
            {
                point.x = temp_pointcloud->points[i].y; //lidar to loam
                point.y = temp_pointcloud->points[i].z;
                point.z = temp_pointcloud->points[i].x;

                if (sqrt(point.x * point.x + point.y * point.y + point.z * point.z)>10){
                    continue;
                }
                point.intensity = temp_pointcloud->points[i].intensity;
                out_pointcloud->push_back(point);
            }
            all_cloudpoint.push_back(out_pointcloud);
            cp_timestamps.push_back(s1->header.stamp.toSec());
        }
    }
    
    //std::cout<<"count cp: "<<all_cloudpoint.size()<<std::endl;
    std::cout<<"count pose: "<<cloudKeyPoses6D->size()<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud;
    global_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud_ds;
    global_cloud_ds.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    int imgcount = 0;
    
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        imgcount++;
//         if (imgcount >10000) break;
        if(simg!=NULL && m.getTopic() == image_topic)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat img_;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(simg, "rgb8");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
            cv_ptr->image.copyTo(img_);
            //all_image.push_back(img_);
            
            int coors_cp_index=-1;
            int scan_cp_index=-1;
            for(int j=0; j<cloudKeyPoses6D->size(); j++){
                if (simg->header.stamp.toSec()>=cloudKeyPoses6D->points[j].time-0.05 && simg->header.stamp.toSec()<=cloudKeyPoses6D->points[j].time+0.05){
                    coors_cp_index=j;
                    //std::cout<<"time diff pose: "<<std::abs(simg->header.stamp.toSec()-cloudKeyPoses6D->points[j].time)<<std::endl;
                    break;
                }
            }
            //std::cout<<"image time : "<<simg->header.stamp.toSec()<<std::endl;
            for(int k=0; k<cp_timestamps.size(); k++){
                //std::cout<<"cloud time : "<<cp_timestamps[k]<<std::endl;
                if (simg->header.stamp.toSec()>=cp_timestamps[k]-0.1 && simg->header.stamp.toSec()<=cp_timestamps[k]+0.1){
                    scan_cp_index=k;
                    //std::cout<<"time diff cloud: "<<std::abs(simg->header.stamp.toSec()-cp_timestamps[k])<<std::endl;
                    break;
                }
            }        
            
            if (coors_cp_index==-1 || scan_cp_index == -1){
                //std::cout<<"miss a pose or img"<<std::endl;
                continue;
            }
            *global_cloud += *transformPointCloud(all_cloudpoint[scan_cp_index], &cloudKeyPoses6D->points[coors_cp_index],img_);            
        }  
            

    }
    
//     pcl::VoxelGrid<pcl::PointXYZRGBA> downSizeFilter;
//     downSizeFilter.setLeafSize(0.04, 0.04, 0.04);
//     downSizeFilter.setInputCloud(global_cloud);
//     downSizeFilter.filter(*global_cloud_ds);
    
    pcl::io::savePCDFile (root_save_folder+ "/point_cloud_merged_color_1.pcd", *global_cloud, true);
                
    return 0;
}