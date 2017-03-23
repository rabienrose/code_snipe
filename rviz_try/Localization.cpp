/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   
 * @brief  
 *
 * Change Log:
 *      Date              Who                   What
 *    2017.01.25        <Wei.Liu>              Created
 *******************************************************************************
 */

// local class header files.
#include "Localization.h"
#include "ErrCode.h"
#include "GlobalMapLoader.h"
#include "UtilityHelper.h"

#include "Frame.h"
#include "MapPoint.h"
#include "GlobalMap.h"

#include "LOCEvent.h"

// system header files.
#include <set>

// other lib header files.
#include <opencv2/opencv.hpp>         // directly using cv definition of cv::KeyPoint, etc.
// #include "features2d.hpp"

// local project header files.
#include "AlgoLog.hpp"
#include "LOCWrapper.h"

#include "FeatureExtractor/FeatureExtractor.hpp"   // using feature extractor.
#include "SlamWrapper.hpp"                         // using wrapper to provide params config to feature extractor.

#include "pose_voting.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace algo { namespace vehicle 
{
    void Localization::testRviz(){
        ros::NodeHandle node;  
    	tf2_ros::StaticTransformBroadcaster tfb;
        ros::Rate rate(1);
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "chamo";
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 10;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        tfb.sendTransform(transformStamped);  
        ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("poses", 1);    
        while (node.ok()){
            geometry_msgs::PoseStamped poseStamp;
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
            rate.sleep();
        }
    }
    
	void Localization::publishTransformations(ros::Publisher& pose_pub, std::vector<LOC::SptrFrame>& allFrame, ros::Publisher& roadmodeB) {
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = "/world";
        for (std::size_t i = 0; i < allFrame.size(); ++i) {
        	LOC::SptrFrame t_Frame = allFrame[i];
        	cv::Point3f posi =t_Frame->getCameraCenter();
        	Eigen::Quaternionf qua = t_Frame->getPoseDir();
        	geometry_msgs::Pose pose;
        	pose.position.x = posi.z;
            pose.position.y = posi.x;
            pose.position.z = -posi.y;
            pose.orientation.x = qua.z();
            pose.orientation.y = qua.x();
            pose.orientation.z = qua.y();
            pose.orientation.w = qua.w(); 
            poses.poses.push_back(pose);  
        }
        pose_pub.publish(poses);
        
        int line_count=0;
        float road_w=2;
        for(int i=-3;i<=3;i++){
            if(i%2==0){
                continue;
            }
            visualization_msgs::Marker roadPaint;
            roadPaint.header.frame_id = "/world";
            roadPaint.header.stamp = ros::Time::now();
            roadPaint.ns = "roadPaint";
            roadPaint.id = line_count;
            roadPaint.type = visualization_msgs::Marker::LINE_STRIP;
            roadPaint.action = visualization_msgs::Marker::ADD;
            roadPaint.pose.position.x = 0;
            roadPaint.pose.position.y = 0;
            roadPaint.pose.position.z = 0;
            roadPaint.pose.orientation.x = 0;
            roadPaint.pose.orientation.y = 0;
            roadPaint.pose.orientation.z = 0;
            roadPaint.pose.orientation.w = 1;
            roadPaint.scale.x = 0.1;
            roadPaint.scale.y = 0.1;
            roadPaint.scale.z = 0.1;
            roadPaint.color.r = 0.0f;
            roadPaint.color.g = 1.0f;
            roadPaint.color.b = 0.0f;
            roadPaint.color.a = 1.0;
            roadPaint.lifetime = ros::Duration();
            
            for(unsigned int j=0;j<allFrame.size()-1;j++){
                cv::Point3f posi1 =allFrame[j]->getCameraCenter();
                cv::Point3f posi2 =allFrame[j+1]->getCameraCenter();
                Eigen::Vector3f f_vec(posi2.x-posi1.x, posi2.y-posi1.y, posi2.z-posi1.z);
                Eigen::Vector3f u_vec(0,1,0);
                Eigen::Vector3f l_vec = f_vec.cross(u_vec);
                l_vec = l_vec/l_vec.norm();
                l_vec= l_vec*road_w*i;
                Eigen::Vector3f roadP= Eigen::Vector3f(posi1.x, posi1.y, posi1.z)+l_vec;
                geometry_msgs::Point p1;
                p1.x = roadP.z();
                p1.y = roadP.x();
                p1.z = -roadP.y()-1;
                //std::cout<<"roadpoints: "<<roadP<<std::endl;
                roadPaint.points.push_back(p1);
            }
            roadmodeB.publish(roadPaint);
            line_count++;
        }
	}
	
    void Localization::publishImg(ros::Publisher& pubImg, LOC::SptrFrame& curFrame, ros::Publisher& pubCam){
        cv::Mat curKfImg = curFrame->getImg().clone();
        const std::vector<LOC::KeyPoint>& curKfKpItems = curFrame->getKeyPoints();
        for(unsigned int j=0;j<curKfKpItems.size();j++){
            cv::KeyPoint kp = curKfKpItems[j].getKp();
            cv::circle(curKfImg, kp.pt*4.0/3.0, 4, cv::Scalar(0,0,255,255),5);
        }
        cv::cvtColor(curKfImg, curKfImg, CV_BGR2BGRA);
        cv::resize(curKfImg,curKfImg,cv::Size(curKfImg.cols/4, curKfImg.rows/4));
        cv::flip(curKfImg, curKfImg,1);
        //std::cout<<"curKfImg.step[0]: "<<curKfImg.step[0]<<std::endl;
        sensor_msgs::Image rviz_img;
        rviz_img.height = curKfImg.rows;
        rviz_img.width = curKfImg.cols;
        rviz_img.encoding = "8UC4";
        unsigned char *p = curKfImg.ptr<unsigned char>(0);
        std::vector<unsigned char> vec(p, p+curKfImg.cols*4*curKfImg.rows);
        rviz_img.data = vec;
        rviz_img.step = curKfImg.cols*4;
        pubImg.publish(rviz_img);
        
        sensor_msgs::CameraInfo rviz_cam;
        rviz_cam.height = curKfImg.rows;
        rviz_cam.width = curKfImg.cols;
        rviz_cam.distortion_model = "plumb_bob";
        for (int i=0;i<5;i++){
            rviz_cam.D.push_back(0);
        }
        double fx = LOC::CamParamWrapper::instance()->fx()/4;
        double fy = LOC::CamParamWrapper::instance()->fy()/4;
        double cx = LOC::CamParamWrapper::instance()->cx()/4;
        double cy = LOC::CamParamWrapper::instance()->cy()/4;
        rviz_cam.K[0]=fx;
        rviz_cam.K[1+1*3]=fy;
        rviz_cam.K[2+0*3]=cx;
        rviz_cam.K[2+1*3]=cy;
        rviz_cam.K[2+2*3]=1;
        cv::Mat pose = cv::Mat(curFrame->getPose());
        for (int i=0;i<3;i++){
            for (int j=0;j<4;j++){
                rviz_cam.P[i*4+j] =pose.at<float>(i,j);
            }
        }
        pubCam.publish(rviz_cam);
    }
	
    void Localization::publishCurrentPose(tf2_ros::TransformBroadcaster& transformB, LOC::SptrFrame& curFrame, ros::Publisher& carmodeB){
        cv::Point3f posi =curFrame->getCameraCenter();
    	Eigen::Quaternionf qua = curFrame->getPoseDir();
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id ="curFrame";//+std::to_string(curFrame->getId());
        transformStamped.transform.translation.x = posi.z;
        transformStamped.transform.translation.y = posi.x;
        transformStamped.transform.translation.z = -posi.y;
        transformStamped.transform.rotation.x = qua.z();
        transformStamped.transform.rotation.y = qua.x();
        transformStamped.transform.rotation.z = qua.y();
        transformStamped.transform.rotation.w = qua.w();
        transformB.sendTransform(transformStamped);
        
        Eigen::Quaternionf rot(0,0.7071,0,0.7071);
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "car";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = posi.z;
        marker.pose.position.y = posi.x;
        marker.pose.position.z = -posi.y;
        qua= qua*rot;
        marker.pose.orientation.x = qua.z();
        marker.pose.orientation.y = qua.x();
        marker.pose.orientation.z = qua.y();
        marker.pose.orientation.w = qua.w();
        marker.scale.x = 1.0;
        marker.scale.y = -1.0;
        marker.scale.z = -1.0;
        //marker.color.r = 0.5f;
        //marker.color.g = 0.5f;
        //marker.color.b = 0.5f;
        //marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker.mesh_use_embedded_materials=true;
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        //marker.mesh_resource = "package://pr2_description/volks.dae";
        marker.mesh_resource = "package://pr2_description/volks.dae";
        carmodeB.publish(marker);
        
        std::vector<LOC::SptrMapPoint> kfMatchPairs = curFrame->getMapPointIndices();
        visualization_msgs::Marker connections;
        connections.header.frame_id = "/world";
        connections.header.stamp = ros::Time::now();
        connections.ns = "connection";
        connections.id = 0;
        connections.type = visualization_msgs::Marker::LINE_LIST;
        connections.action = visualization_msgs::Marker::ADD;
        connections.pose.position.x = 0;
        connections.pose.position.y = 0;
        connections.pose.position.z = 0;
        connections.pose.orientation.x = 0;
        connections.pose.orientation.y = 0;
        connections.pose.orientation.z = 0;
        connections.pose.orientation.w = 1;
        connections.scale.x = 0.1;
        connections.scale.y = 0.1;
        connections.scale.z = 0.1;
        connections.color.r = 1.0f;
        connections.color.g = 1.0f;
        connections.color.b = 1.0f;
        connections.color.a = 0.5;
        connections.lifetime = ros::Duration();
        for(unsigned int j=0;j<kfMatchPairs.size();j++){
            geometry_msgs::Point p1;
            p1.x = posi.z;
            p1.y = posi.x;
            p1.z = -posi.y;
            geometry_msgs::Point p2;
            p2.x = kfMatchPairs[j]->getPosition().z;
            p2.y = kfMatchPairs[j]->getPosition().x;;
            p2.z = -kfMatchPairs[j]->getPosition().y;; 
            connections.points.push_back(p1);
            connections.points.push_back(p2);
        }
        carmodeB.publish(connections);
    }

	void Localization::publishPointCloud(ros::Publisher& cloud_pub) {
	    std::vector<LOC::SptrFrame> allFrame;
        LOC::GlobalMap::instance()->getAllFrame(allFrame);
        //std::cout<<"frame size:"<<allFrame.size()<<std::endl;
        for(unsigned int i=0;i<allFrame.size();i++){
            //std::cout<<"frame iid:"<<i<<std::endl;
            LOC::SptrFrame t_frame = allFrame[i];
            const std::vector<LOC::KeyPoint>& curKfKpItems = t_frame->getKeyPoints();
            std::vector<LOC::SptrMapPoint> kfMatchPairs = t_frame->getMapPointIndices();
            cv::Mat curKfImg = t_frame->getImg();
            if(curKfImg.type() == CV_8UC1){
                cv::cvtColor(curKfImg, curKfImg, cv::COLOR_GRAY2BGR);
                //std::cout<<"CV_8UC1"<<std::endl;
            }
            else if(curKfImg.type() == CV_8UC4){
                //std::cout<<"CV_8UC4"<<std::endl;
            }
            else if(curKfImg.type() == CV_8UC3){
                //std::cout<<"CV_8UC3"<<std::endl;
            }
            else{
                //std::cout<<"none"<<std::endl;
            }
            //std::cout<<"kp count:"<<curKfKpItems.size()<<std::endl;
            //std::cout<<"mp count:"<<kfMatchPairs.size()<<std::endl;
            //std::cout<<"img size:"<<curKfImg.rows<<":"<<curKfImg.cols<<std::endl;
            for(unsigned int j=0;j<curKfKpItems.size();j++){
                cv::KeyPoint kp = curKfKpItems[j].getKp();
                //std::cout<<"mp uv:"<<kp.pt<<std::endl;
                int v =round(kp.pt.y/3.0*4.0);
                int u =round(kp.pt.x/3.0*4.0);
                if(v >=0 && v< curKfImg.rows && u >=0 &&u <curKfImg.cols){
                    cv::Vec3b color = curKfImg.at<cv::Vec3b>(v, u);
                    //std::cout<<"mp color:"<<color<<std::endl;
                    kfMatchPairs[j]->color = color;
                }else{
                    std::cout<<"uv out of img"<<std::endl;
                }
            }
        }
        
	    std::vector<LOC::SptrMapPoint> valueList;
	    LOC::GlobalMap::instance()->getAllMapPoint(valueList);
	    sensor_msgs::PointCloud cloud;
	    cloud.header.frame_id = "/world";
	    cloud.points.resize(valueList.size());
	    cloud.channels.resize(3);
//	    cloud.channels[0].name = "intensities";
//	    cloud.channels[0].values.resize(valueList.size());
	    cloud.channels[0].name = "r";
	    cloud.channels[0].values.resize(valueList.size());
	    cloud.channels[1].name = "g";
	    cloud.channels[1].values.resize(valueList.size());
	    cloud.channels[2].name = "b";
	    cloud.channels[2].values.resize(valueList.size());
	    //cloud.channels[0].name = "color";
	    //cloud.channels[0].values.resize(valueList.size());
	    for (std::size_t i = 0; i < valueList.size(); ++i) {
	        cv::Point3f posi = valueList[i]->getPosition();
	        cloud.points[i].x = posi.z;
	        cloud.points[i].y = posi.x;
	        cloud.points[i].z = -posi.y-1;
//	        cloud.channels[0].values[i] = valueList[i]->color[2]/255.0;
	        cloud.channels[0].values[i] = valueList[i]->color[2]/255.0;
	        cloud.channels[1].values[i] = valueList[i]->color[1]/255.0;
	        cloud.channels[2].values[i] = valueList[i]->color[0]/255.0;
	        //std::cout<<"point color: "<<valueList[i]->color<<std::endl;
	        //int rgb = valueList[i]->color[2]*0x10000 +valueList[i]->color[1]*0x100 +valueList[0]->color[2]; 
//	        int rgb = 0xff00;
//	        float float_rgb = *reinterpret_cast<float*>(&rgb);
//	        cloud.channels[0].values[i] = float_rgb;
	    }
	    cloud_pub.publish(cloud);
	}
	
    void Localization::publishRootFrame(tf2_ros::StaticTransformBroadcaster& transformB, Eigen::Vector3f posi){
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id ="chamo";
        transformStamped.transform.translation.x = posi.z();
        transformStamped.transform.translation.y = posi.x();
        transformStamped.transform.translation.z = -posi.y();
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.z();
        transformStamped.transform.rotation.y = q.x();
        transformStamped.transform.rotation.z = q.y();
        transformStamped.transform.rotation.w = q.w();
        transformB.sendTransform(transformStamped);
    }

    // using wrapper to store any pointer which we need to use.
    LOC::ErrCode Localization::init(algo::ICameraParam  *pICamParam,
                         algo::ISlamConfig   *pISlamCfg,
                         algo::ISlamConfig   *pIModelCfg,
                         algo::IImage        *pIImg,
                         algo::IGps    *pIGPS,
                         algo::IKeyFrameOut  *pIKFs,
                         algo::IRTMatrixOut  *pRTMat,
                         std::string    strVoc,
                         std::string    debugFolder, /*= "/"*/
                         int            startFrmIdx/* = 0*/,
                         int            totalFrame /*= 10000*/)
    {
        using namespace algo::vehicle::LOC;

        ErrCode ret = ERRCODE_LOC_OK;
        
        // TODO:  to make use of keyframe out and rt matrix out.
        (void)pIKFs;
        (void)pRTMat;
        (void)strVoc;      // used for BOW file.
        (void)debugFolder; // todo: to make use of debug folder.
        
        // initialize for localization, we use global wrapper for convinience.
        LOC::CamParamWrapper::instance(pICamParam);        // split wrapper into different ones.
        LOC::ParamCfgWrapper::instance(pISlamCfg);       
        LOC::ModelCfgWrapper::instance(pIModelCfg);       
        LOC::ImgReaderWrapper::instance(pIImg);       
        LOC::GpsReaderWrapper::instance(pIGPS);  
        // TODO: current arch is not very good, visualizer depends on evt dispatcher, and vice versa.
        // evt dispatcher must be created before other things.
        LOC::EvtDispatchWrapper::instance(new EvtDispatcher());       
        //LOC::VisualizerWrapper::instance(visualizerFactory())->registry();
        // 
        // fe::SlamWrapper::setParamConfig(pISlamCfg);

        debugFolder_ = debugFolder;
        startFrmIdx_ = startFrmIdx;
        totalFrame_  = totalFrame;

        // for localization, we need to load global 3d-model into memory.
        LOC::GlobalMapLoader loader;
        loader.load();
        
        // load the DBoW3 vacabulary
        //global_Voc.load(strVoc);

        // initialize for the extract features.
        pFeatureExtractor_ = std::unique_ptr<fe::FeatureExtractor>(new fe::FeatureExtractor());
        if(!pFeatureExtractor_)
        {
            LOG_ERROR << "can not create feature extractor";
            return ERRCODE_LOC_FAILURE;
        }

        bool isInited = pFeatureExtractor_->init(LOC::ParamCfgWrapper::instance());
        if(!isInited)
        {
            LOG_ERROR << "feature extractor can not be initialized.";
            return ERRCODE_LOC_FAILURE;
        }
        

        return ret;

    }
    
    LOC::ErrCode Localization::locate()
    {
        ros::NodeHandle n;
        using namespace algo::vehicle::LOC;
        // check validation.
        if(!LOC::ImgReaderWrapper::instance())
        {
            LOG_ERROR << "null img reader.";
            return ERRCODE_LOC_FAILURE;
        }
        
        // locate for every img.
        int nCurFrameIdx = startFrmIdx_;

        // amount of frames to process
        int totalFramesToProcess = 0;

        if ( LOC::ImgReaderWrapper::instance() )
        {
            // this operation is based on the premise that frame index starts at zero
            // if this is not the case, then totalFramesToProcess is incorrect
            totalFramesToProcess = LOC::ImgReaderWrapper::instance()->getNum() - startFrmIdx_;
        }
        totalFramesToProcess = std::min(totalFrame_, totalFramesToProcess);
        std::vector<LOC::SptrFrame> allFrame;
        LOC::GlobalMap::instance()->getAllFrame(allFrame);
	    
        ErrCode ret = ERRCODE_LOC_OK;
        cv::Mat img;  
        //double timeStamp = 0.0f;
        //testRviz();
        ros::Rate rate(5);
        ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1); 
        ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("car", 1);
        ros::Publisher painting_pub = n.advertise<visualization_msgs::Marker>("painting", 1);
        ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("cur/image", 1);
        ros::Publisher cam_pub = n.advertise<sensor_msgs::CameraInfo>("cur/camera_info", 1);
        tf2_ros::StaticTransformBroadcaster tfb;
        tf2_ros::TransformBroadcaster curFrameB;
        for (unsigned int i=0; i<allFrame.size() && n.ok(); i++){
            LOC::SptrFrame t_Frame = allFrame[i];
            cv::Point3f posi =t_Frame->getCameraCenter();
            if(i==0){
                publishRootFrame(tfb, Eigen::Vector3f(posi.x, posi.y, posi.z));
            }
            if(i<=2){
                publishTransformations(pose_pub, allFrame, painting_pub);
                publishPointCloud(pointcloud_pub);
            }
            publishImg(img_pub, t_Frame, cam_pub);
            publishCurrentPose(curFrameB, t_Frame, marker_pub);
            //std::cout<<"curFrameId: "<<i<<std::endl;
            rate.sleep();
        }
            
        while (nCurFrameIdx < (startFrmIdx_ + 10))
        {
            if(!LOC::ImgReaderWrapper::instance()->get(nCurFrameIdx, img))
            {
                LOG_ERROR << "can not get img. [idx=" << nCurFrameIdx << "]";
                return ERRCODE_IMG_ERROR;
            }

            // read gps info if any.
            cv::Point3f relGps;
            if(LOC::GpsReaderWrapper::instance())
            {
                LOC::GpsReaderWrapper::instance()->getRelPos(nCurFrameIdx, relGps);
            }
            
            //======================================================================
            // step0: first of all, cvt current rel-gps into global coordinate.
            PointRelGps newRelGps(relGps);
            if(LOC::GlobalMap::instance())
            {
                LOC::GlobalMap::instance()->cvtLocalGpsIntoGlobal(relGps, newRelGps);
            }
        
            //int ret = locate(nCurFrameIdx, timeStamp, img, newRelGps);
            // if(ERRCODE_LOC_OK != ret)
            // {
            //     // to make locating more robust, continue if loc failed.
            //     continue;
            // }

            nCurFrameIdx++;
        }

        return ret;

    }

} } // namespace algo::vehicle.

