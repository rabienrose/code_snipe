
#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>

namespace chamo
{
    class DataInterface{
    public:
        virtual void AddFrame(int id, std::string name, cv::Mat pose, int type, int kpCount){};
        virtual void AddKepPoint(int frameId, cv::Point2f uv, int octave, std::vector<int> MPIDs){};
        virtual void AddMapPoint(int id, cv::Point3f posi, int isBad){};
        virtual void AddTrack(int mpId, int frameId, int kpId){};
        virtual void ClearData(){};
        virtual void SaveData(std::string fileName){};
    };
}
