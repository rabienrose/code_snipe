#ifndef _COMMON_ALGO_H_
#define _COMMON_ALGO_H_

#include "AlgoInterfaceImp.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "algoInterface/IGps.h"

namespace bf = boost::filesystem;

#include "algoInterface/KeyFrameDef.h"

namespace CommonAlgo{
    class EpipolarFilter
    {
    public:
        EpipolarFilter();
        
        void setK(cv::Mat k);
        bool setPts(const std::vector<cv::Point2f>& pts1
                    , const std::vector<cv::Point2f>& pts2);
        
        bool operator()();
        std::vector<bool>& getInliers(){return inliers_;}
        
    private:
        bool _checkInlier(const cv::Point2f& p1
                          , const cv::Point2f& p2
                          , const cv::Point2f& e);
        
    private:
        cv::Point2f epipole_;
        std::vector<bool> inliers_;
        std::vector<cv::Point2f> pts1_;
        std::vector<cv::Point2f> pts2_;
    };
    
    void computeThreeMaxima(std::vector<std::vector<int>>& histo, const int L, int &ind1, int &ind2, int &ind3);
    void computeEpipolarline(const cv::Point2f &pt, const cv::Mat &F12, float &a, float &b, float &c);
    bool checkDistEpipolarLine(const cv::Point2f &pt, float sigma, const float &a, const float &b, const float &c, const float &denInverse);
    double getlevelSigma(double octave, double sf);
    double predictScaleLevel(double d, double l0_d, double sf);
    bool checkScaleLevel(double d1, int octave1, double sf1, double d2, int octave2, double sf2, double thr = 0.8);
	bool assurePathExists(const std::string &strPath);

	//some functional func
	class transformTools
	{
	private:
		transformTools();
		void convertToAbsGps(cv::Point3d refGPSP3d, algo::GPSData &gpsData);
		void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
		bool CorrectRotation(cv::Mat &R, double &scale, cv::Mat &normR);
        static std::unique_ptr<CommonAlgo::transformTools>  m_pInstance;
		
		algo::IGps* pIGPS_;
		cv::Point3d refgps_;

	public:		
        static CommonAlgo::transformTools* createInstance();
		void convertToAbsCamCenter(std::vector<cv::Mat> &vecCamCenters);
		void convertToAbsGps(cv::Point3d refGPSP3d, std::vector<algo::GPSData> &vecGps);
		void Slam2Gps(cv::Mat &Transform, std::vector<cv::Mat> &vCameraPose);
		bool computeSim3(std::vector<cv::Mat>& vCameraCenter, 
								std::vector<cv::Mat>& vGps,
								cv::Mat &Transform);
	};
}

#endif

