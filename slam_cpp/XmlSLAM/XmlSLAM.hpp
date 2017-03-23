
#pragma once

#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <opencv2/core.hpp>

namespace chamo
{
    class KeyPoint: public cv::KeyPoint{
    public:
        std::vector<int> MPIDs;
        cv::Point2f uv;
        int octave;
        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);
    };
    
    class Frame{
    public:
        int id;
        std::string name;
        cv::Mat pose;
        int type;
        std::vector<KeyPoint> kps;
        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);

    };
    
    class MapPoint{
    public:
        int id;
        int isBad;
        cv::Point3f posi;
        std::vector<std::pair<int,int>> tracks;
        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);
    };
    
    class XmlSLAM
    {
    public:
        void SaveData(std::string fileName);
        void ReadData(std::string fileName);
        void ClearData();
        
        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);
        
        std::map<int, Frame> frames;
        std::unordered_map<int, MapPoint> mapoints;

        
    };
}
