#include <vector>
#include <opencv2/opencv.hpp>

namespace CommonAlgo{
    struct similarityInfo
    {
        float sim1;
        float sim2;
        float sim3;
        float sim4;
        float sim5;
        float sim6;
        float sim7;
        float sim8;
        float sim9;
        float sim0;
    };
    
    bool computeTransform(int i, std::vector<cv::Point3f>& vmRelGps, std::vector<cv::Mat>& vCameraPose, cv::Mat& transform, similarityInfo& score);
    bool computeSim3(std::vector<cv::Mat>& vCameraCenter, std::vector<cv::Mat>& vGps,cv::Mat &Transform);
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
    void Slam2Gps(cv::Mat &Transform, std::vector<cv::Mat> &vCameraPose);
    bool CorrectRotation(cv::Mat &R, double &scale, cv::Mat &normR);
    void computeSimilarity(const std::vector<cv::Mat>& vGps, const std::vector<cv::Mat>& vCameraPose, similarityInfo& si);
    void saveSlamGpsKML(int counter,std::vector<cv::Mat>& vGps,std::vector<cv::Mat>& vCameraPose);
    
}
