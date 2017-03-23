#pragma once
#include "DataInterface.hpp"
#include <string>
#include <iostream>

namespace chamo
{
    class XmlSLAM;
    class DataInterfaceImp: public DataInterface{
    public:
        DataInterfaceImp();
        virtual void AddFrame(int id, std::string name, cv::Mat pose, int type, int kpCount);
        virtual void AddKepPoint(int frameId, cv::Point2f uv, int octave, std::vector<int> MPIDs);
        virtual void AddMapPoint(int id, cv::Point3f posi, int isBad);
        virtual void AddTrack(int mpId, int frameId, int kpId);
        virtual void ClearData();
        virtual void SaveData(std::string fileName);
        XmlSLAM* xmlObj;
    };

}
