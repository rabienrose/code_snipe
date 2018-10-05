#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <map>

void findMax(std::map<int,cv::Point2f>& traj, float& maxX, float& maxY, float& minX, float& minY ){
    maxX = -1e10;
    minX = 1e10;
    maxY = -1e10;
    minY = 1e10;
    std::map<int,cv::Point2f>::iterator it;
    for (it=traj.begin();it!= traj.end();it++){
        if (maxX < it->second.x){
            maxX = it->second.x;
        }
        if (maxY < it->second.y){
            maxY = it->second.y;
        }
        if (minX > it->second.x){
            minX = it->second.x;
        }
        if (minY > it->second.y){
            minY = it->second.y;
        }
    }
}

cv::Mat drawTraj(int reso, std::map<int,cv::Point2f>& traj, cv::Scalar color){
    int margin = 20;
    float w = reso;
    float h = reso;
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
        h = trajH * scale+margin;
        offsetY =margin/2;
    }else{
        scale = h/trajH;
        w = trajW * scale+margin;
        offsetX =margin/2;
    }
    cv::Mat img = cv::Mat(h, w, CV_64FC4, cv::Scalar(1,1,1,1));
    std::map<int,cv::Point2f>::iterator it;
    for (it=traj.begin();it!= traj.end();it++){
        it->second.x = (it->second.x - minX) * scale ;
        it->second.x = it->second.x + offsetX;
        if (it->second.x <=0){
            it->second.x =0;
        }
        if (it->second.x >w-1){
            it->second.x =w-1;
        }
        it->second.y = (it->second.y - minY) * scale;
        it->second.y = it->second.y + offsetY;
        if (it->second.y <=0){
            it->second.y =0;
        }
        if (it->second.y >=h-1){
            it->second.y =h-1;
        }
        img.at<cv::Scalar>(round(it->second.y), round(it->second.x)) = color;
    }
    
    int startFrame = traj.begin()->first;
    int endFrame = traj.rbegin()->first;
    
    int nodeCount = 5;
    cv::Scalar colorList[5] = {{1,0,0,1}, {0,1,0,1}, {0,0,1,1}, {0,0.8,0.8,1}, {0.8,0,0.8,1}};
    for (int i=1; i<=nodeCount;i++){
        int curInd = startFrame + (endFrame - startFrame)*i/(float)(nodeCount+1);
        cv::Point2f pt;
        for (int j=0;j<50;j++){
            if (traj.count(curInd) ==0){
                curInd--;
            }else{
                pt = traj[curInd];
                break;
            }
        }
        cv::circle(img, cv::Point(round(pt.x), round(pt.y)), 2, colorList[i-1], 2);
    }
    return img;
}

cv::Mat combineImgs(cv::Mat img1, cv::Mat img2){
    cv::Mat img1_r = img1;
    cv::Mat img2_r = img2;
    if (img1.cols>img1.rows){
        img1_r = img1_r.t();
    }
    if (img2.cols>img2.rows){
        img2_r = img2_r.t();
    }
    int combineW = img1_r.cols+img2_r.cols;
    int combineH = std::max(img1_r.rows, img2.rows);
    cv::Mat img = cv::Mat(combineH, combineW, CV_64FC4, cv::Scalar(1,1,1,1));
    img1_r.copyTo(img.rowRange(0, img1_r.rows).colRange(0, img1_r.cols));
    img2_r.copyTo(img.rowRange(0, img1_r.rows).colRange(img1_r.cols, img1_r.cols + img2_r.cols));
    
    return img;
}

cv::Mat putText(cv::Mat img, std::vector<std::string> text, std::vector<cv::Scalar> colors){
    cv::Mat reImg;
    int bottomText=0;
    int leftText=0;
    if (img.cols>img.rows){
        reImg = cv::Mat(img.rows+50, img.cols, CV_64FC4, cv::Scalar(1,1,1,1));
        bottomText = img.rows+40;
        leftText = 10;
    }else{
        reImg = cv::Mat(img.rows, img.cols+150, CV_64FC4, cv::Scalar(1,1,1,1));
        bottomText = img.rows -10;
        leftText = img.cols+10;
    }
    img.copyTo(reImg.rowRange(0, img.rows).colRange(0, img.cols));
    for (int i=0;i<text.size();i++){
        cv::putText(reImg, text[i].c_str(), cv::Point(leftText,bottomText-20*i), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i]);
    }
    return reImg;
}

int main(int argc, char **argv){
    std::string gpsAddr = argv[1];
    std::string slamReAddr = argv[2];
    std::string caseName = argv[3];
    std::string outputRoot = argv[4];
    
    cv::Mat img = cv::Mat(512, 512, CV_64FC4, cv::Scalar(1,1,1,1));
    
    
    //===========================================================//
    //                          SLAM                             //
    //===========================================================//
    std::fstream slamTxt;
    slamTxt.open(slamReAddr, std::ios::in);
    std::map<int,cv::Point2f> slamTrajMap;
    if (slamTxt.is_open())
    {
        while (!slamTxt.eof())
        {
            char lineData[256];
            slamTxt.getline(lineData, 256);
            cv::Point2f pt;
            std::string line(lineData);
            if (line != "none"){
                int id;
                sscanf(lineData, "%d, %f, %f",&id, &pt.y, &pt.x);
                slamTrajMap[id] = pt;
            }
        }
    }
    int startFrame = slamTrajMap.begin()->first;
    int endFrame = slamTrajMap.rbegin()->first;
    cv::Mat slam_traj = drawTraj(512, slamTrajMap, cv::Scalar(0,0,1,1));
    //===========================================================//
    
    //===========================================================//
    //                          GPS                              //
    //===========================================================//
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
    std::map<int,cv::Point2f> gpsTrajMap;
    for (int i=startFrame; i<=endFrame; i++){
        gpsTrajMap[i] = cv::Point2f(vmRelGps[i].x, vmRelGps[i].y);
    }
    cv::Mat gps_traj = drawTraj(512, gpsTrajMap, cv::Scalar(0,0,0,1));
    //===========================================================//
    
    cv::Mat combineImg = combineImgs(gps_traj, slam_traj);
    std::vector<std::string> texts;
    texts.push_back("GPS");
    texts.push_back("SLAM");
    std::stringstream ss;
    ss<<startFrame<<"-"<<endFrame;
    texts.push_back(ss.str());
    texts.push_back("Frame:");
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(0,0,0,1));
    colors.push_back(cv::Scalar(0,0,1,1));
    colors.push_back(cv::Scalar(0,0,0,1));
    colors.push_back(cv::Scalar(0,0,0,1));
    cv::Mat textImg = putText(combineImg, texts, colors);
    cv::Mat img4Save;
    img4Save = textImg*255;
    img4Save.convertTo(img4Save, CV_8UC4);
    cv::cvtColor(img4Save, img4Save, CV_BGRA2BGR);
    if (img4Save.type() != CV_8UC3){
        return -1;
    }
    cv::imwrite(outputRoot +caseName+"_traj.png", img4Save);
    //cv::imshow("all", textImg);
    //cv::waitKey(0);
    return 0;
}
