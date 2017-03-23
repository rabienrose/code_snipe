#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unordered_map>

struct mxArray_tag;

namespace chamo {
    class ViewInterfaceImp;
}


namespace ygomi
{
    class GlobalMap;
    class MapPoint;
  
    class CoreData
    {
    public:
        void init();
        void SaveVisFile(std::string fileAddr);
        void SaveMat(std::string fileAddr);
        void AddFrame(long frameId, cv::Mat pose, std::string name, bool isKF);
        void AddKeyPoint(long frameId, cv::KeyPoint kp, cv::Mat desc, std::vector<long> mpList);
        void AddMappoint(long mpId, cv::Point3f posi, cv::Mat desc, bool isBad);
        void AddTrack(long mpId, long frameId, int kpId);
        
    protected:
        GlobalMap* m_pGlobalMap;
        int fps;
        chamo::ViewInterfaceImp* viewerHandler;
    };
}