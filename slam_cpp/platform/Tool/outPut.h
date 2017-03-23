#ifndef OUT_PUT_H
#define OUT_PUT_H

#include "typeDef.h"

#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>

#include <opencv2/opencv.hpp>

namespace ygomi
{
    class outPut
    {
    private:
		outPut(std::string &debugFolder);
		bool assurePathExists(const std::string &strPath);
		
        //Singleton working instance.
        static std::unique_ptr<ygomi::outPut>  m_pInstance;
		std::string debugFolder_;
    public:
		~outPut();
        static ygomi::outPut* createInstance(std::string &debugFolder);
		
		bool generateSlamKml(const std::string &strFileName, const std::vector<cv::Mat> &vecTrajectory);
		void saveSlamGpsKML(long begin, long end, std::vector<cv::Mat>& vGps,std::vector<cv::Mat>& vCameraPose);
		
    };

} //namespace ORB_SLAM

#endif // OUT_PUT_H

