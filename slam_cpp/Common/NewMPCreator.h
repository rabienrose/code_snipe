#pragma once
#include <opencv2/opencv.hpp>

namespace algo{
    class NewCloudPointCreator
    {
    public:
        NewCloudPointCreator(cv::Mat& obPose, cv::Mat K, float _scaleFactor, int _descType,const std::vector<float>& scaleFactors,
                             std::vector<cv::KeyPoint>& _obKps, std::vector<cv::Mat>& _obDescs, std::vector<bool>& _obMpMask);
        
        void set(cv::Mat obPose, std::vector<cv::KeyPoint>& _refKps, std::vector<cv::Mat>& _refDescs, std::vector<bool>& _refMpMask);
        bool operator()(std::vector<std::pair<size_t, size_t> >& matchedPairs, std::vector<cv::Point3f>& posiList);
        
    public:
        
        int SearchForTriangulation(std::vector<std::pair<size_t,size_t> >& vMatchedIndices);
        
        std::vector<std::pair<std::size_t, std::size_t> > _getMatchPair();
        
        std::vector<std::pair<std::size_t, std::size_t> >__filter(std::vector<std::pair<std::size_t, std::size_t> >& mIdx);
        
        cv::Mat _computeF12(cv::Mat pose1, cv::Mat pose2);
        
        bool _checkIsFarPoint(const cv::Mat& m1, const cv::Mat& R1
                              , const cv::Mat& m2, const cv::Mat& R2);
        
        bool _checkIsX3DValid(const cv::Mat& x3D
                              , const cv::Mat& T, const cv::Mat& k
                              , const cv::Point2f& pt
                              , int octave, float sf);
        
        bool _checkIsValidOctave(const cv::Mat& x3D
                                 , int octave_r
                                 , int octave_ob
                                 , float thresh = 1.9);
        
        cv::Mat _createx3D2(
                            std::size_t idx1
                            ,std::size_t idx2);
        
        
    private:
        int rkf_;
        int obkf_;
        
        cv::Mat K;
        
        cv::Mat K_inv;
        
        cv::Mat rT_;
        cv::Mat obT_;
        
        cv::Mat rR_;
        cv::Mat obR_;
        
        cv::Mat rPose;
        cv::Mat obPose;
        
        cv::Mat F12;
        
        std::vector<cv::Mat> refDescs;
        std::vector<cv::Mat> obDescs;
        
        std::vector<cv::KeyPoint> refKps;
        std::vector<cv::KeyPoint> obKps;
        
        std::vector<bool> refMpMask;
        std::vector<bool> obMpMask;
        
        std::vector<float> scaleFactors;
        float scaleFactor;
        int descType;
        int HISTO_LENGTH =30;
        bool mbCheckOrientation=false;
        
        float cx;
        float cy;
        float fx;
        float fy;
        float invfx;
        float invfy;
    };
}