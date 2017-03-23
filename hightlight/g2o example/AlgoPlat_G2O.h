#ifndef __AlgoPlat_G2O_H__
#define __AlgoPlat_G2O_H__
#include "AlgoPlat_Default.h"
namespace g2o{
    class EdgeSE3ProjectXYZOnlyPose;
    class EdgeSE3ProjectXYZ;
}

namespace chamo{
    class AlgoPlat_G2O: public AlgoPlat_Default{
    public:
        //pose optimization, no outlier remove function, fix optimization iteration
        void optimizeTOneF(int frameId);
        //internal function used by optimizeTOneF
        float PoseOptimize(std::vector<cv::Mat>& mpList, std::vector<cv::KeyPoint>& kpList, cv::Mat& PoseW, cv::Mat K, cv::Mat gpsPosi);
        
        //BA, no outlier remove function, fix optimization iteration
        float doBA(int startKF, int endKF, bool showErr = false);
        float calProjError();
        float calBAError(std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> &edgeList);
        float calBAError(std::vector<g2o::EdgeSE3ProjectXYZ*> &edgeList);
        
        
    };
}


#endif