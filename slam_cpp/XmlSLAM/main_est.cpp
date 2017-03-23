#include "XmlSLAM.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cmath>

void normalizeTraj(std::vector<cv::Point2f>& traj){
    if (traj.size() <=10){
        std::cout<<"too few points"<<std::endl;
        return;
    }
    float offsetX = traj[0].x;
    float offsetY = traj[0].y;
    for (int i=0;i<traj.size();i++){
        if (traj[i].x == MAXFLOAT){
            continue;
        }
        traj[i].x = traj[i].x - offsetX;
        traj[i].y = traj[i].y - offsetY;
    }
    
    float dirX = traj[1].x;
    float dirY = traj[1].y;
    float theta = std::atan2(dirY, dirX);
    
    for (int i=0;i<traj.size();i++){
        if (traj[i].x == MAXFLOAT){
            continue;
        }
        float theta_n = -theta;
        float x_new = std::cos(theta_n) * traj[i].x - std::sin(theta_n) * traj[i].y;
        float y_new = std::sin(theta_n) * traj[i].x + std::cos(theta_n) * traj[i].y;
        traj[i].x = x_new;
        traj[i].y = y_new;
    }
}

void findMax(std::vector<cv::Point2f>& traj, float& maxX, float& maxY, float& minX, float& minY ){
    maxX = -MAXFLOAT;
    minX = MAXFLOAT;
    maxY = -MAXFLOAT;
    minY = MAXFLOAT;
    for (int i=0;i<traj.size();i++){
        if (traj[i].x == MAXFLOAT){
            continue;
        }
        if (maxX < traj[i].x){
            maxX = traj[i].x;
        }
        if (maxY < traj[i].y){
            maxY = traj[i].y;
        }
        if (minX > traj[i].x){
            minX = traj[i].x;
        }
        if (minY > traj[i].y){
            minY = traj[i].y;
        }
    }
}

void drawTraj(cv::Mat& ori, std::vector<cv::Point2f>& traj, cv::Scalar color){
    float w = ori.cols;
    float h = ori.rows;
    float maxX = 0;
    float minX = 0;
    float maxY = 0;
    float minY = 0;
    findMax(traj, maxX, maxY, minX, minY);
    float trajW = maxX- minX;
    float trajH = maxY- minY;
    float scale=1;
    float offsetX=0;
    float offsetY=0;
    if (trajW/trajH >w/h){
        scale = w/trajW;
        offsetY = h/2/scale-trajH/2;
    }else{
        scale = h/trajH;
        offsetY = w/2/scale-trajW/2;
    }
    for (int i=0;i<traj.size();i++){
        if (traj[i].x == MAXFLOAT){
            continue;
        }
        traj[i].x = traj[i].x - minX + offsetX;
        traj[i].x = traj[i].x * scale;
        if (traj[i].x <=0){
            traj[i].x =0;
        }
        if (traj[i].x >w-1){
            traj[i].x =w-1;
        }
        traj[i].y = traj[i].y - minY + offsetY;
        traj[i].y = traj[i].y * scale;
        if (traj[i].y <=0){
            traj[i].y =0;
        }
        if (traj[i].y >=h-1){
            traj[i].y =h-1;
        }
        ori.at<cv::Scalar>(round(traj[i].y), round(traj[i].x)) = color;
    }
}

int main(int argc, char **argv){
    std::string gpsAddr = argv[1];
    std::string slamReAddr = argv[2];
    int startFrame = atoi(argv[3]);
    int endFrame = atoi(argv[4]);
    
    cv::Mat img = cv::Mat(512, 512, CV_64FC4, cv::Scalar(1,1,1,1));
    
    std::vector<cv::Point3f> vmRelGps;
    std::fstream fGps;
    fGps.open(gpsAddr.c_str(), std::ios::in);
    if (fGps.is_open())
    {
        while (!fGps.eof())
        {
            char lineData[256];
            fGps.getline(lineData, 256);
            cv::Point3f relGPS;
            float timeStamp;
            sscanf(lineData, "$GPGGA,%f,%f,%f,%f",&timeStamp ,&relGPS.x, &relGPS.y, &relGPS.z);
            vmRelGps.push_back(relGPS);
        }
    }
    fGps.close();
    
    std::vector<cv::Point2f> gpsPosi;
    gpsPosi.reserve(endFrame - startFrame);
    for (int i=startFrame; i<endFrame; i++){
        gpsPosi.push_back(cv::Point2f(vmRelGps[i].x, vmRelGps[i].y));
    }
    normalizeTraj(gpsPosi);
    drawTraj(img, gpsPosi, cv::Scalar(0,0,1,1));
    
    chamo::XmlSLAM xmlObj;
    xmlObj.ReadData(slamReAddr);
    std::vector<cv::Point2f> slamPosi;
    slamPosi.reserve(xmlObj.frames.size());
    typedef std::map<int, chamo::Frame>::iterator it_type;
    for(it_type iterator = xmlObj.frames.begin(); iterator != xmlObj.frames.end(); iterator++) {
        if(iterator->second.type == 2){
            cv::Mat pose = iterator->second.pose;
            cv::Mat Rcw = pose.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pose.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat C = -Rwc*tcw;
            if (C.type() != CV_32FC1){
                C.convertTo(C, CV_32FC1);
            }
            cv::Point2f pt;
            pt.x = C.at<float>(0);
            pt.y = C.at<float>(2);
            slamPosi.push_back(pt);
        }else{
            slamPosi.push_back(cv::Point2f(MAXFLOAT,MAXFLOAT));
        }
        
    }
    normalizeTraj(slamPosi);
    drawTraj(img, slamPosi, cv::Scalar(0,1,0,1));
    cv::imshow("chamo", img);
    cv::waitKey(0);
    return 0;
}
