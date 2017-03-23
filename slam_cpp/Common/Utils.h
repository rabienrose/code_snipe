#pragma once
#include <string>
#include <opencv/cv.h>
#include <Eigen/Dense>
#include"g2o/g2o/types/types_six_dof_expmap.h"
#include"g2o/g2o/types/types_seven_dof_expmap.h"
#include <boost/random.hpp>

namespace algo
{
    cv::Mat SkewSymmetry(cv::Mat v);
    
    //find the minimum value and sub minimum value
    template<typename _T>
    class Min2Compare
    {
    public:
        Min2Compare(_T def, int defIdx =-1){
            min1_ = def;
            min2_ = def;
            idx1_ = defIdx;
            idx2_ = defIdx;
        }
        void operator()(_T v, int idx){
            if(v< min1_){
                min2_ = min1_;
                idx2_ = idx1_;
                min1_ = v;
                idx1_ = idx;
            }
            else if(v< min2_){
                min2_=v;
                idx2_ = idx;
            }
        }
        
        _T getMin1(){return min1_;}
        _T getMin2(){return min2_;}
        int getMinIdx1(){return idx1_;}
        int getMinIdx2(){return idx2_;}
        
    private:
        _T min1_;
        int idx1_;
        _T min2_;
        int idx2_;
    };
    
    class UniformRandom{
    public:
        UniformRandom(){
        }
        void reset(int min, int max){
            dist_ = boost::random::uniform_int_distribution<>(min,max);
        }
        int gen()
        {
            return dist_(rng_);
        }
        std::vector<int> get_unique(std::size_t n){
            std::vector<int> vs;
            std::set<int> s;
            while(vs.size() < n){
                int v = gen();
                if(s.find(v) != s.end() )
                    continue;
                
                vs.push_back(v);
                s.insert(v);
            }
            return vs;
        }
        
    private:
        boost::random::mt19937 rng_;
        boost::random::uniform_int_distribution<> dist_;
    };

    class Mean
    {
    public:
        Mean():m_(0),n_(0){}

        void append(double v){
            if(n_){
                m_ = m_ + v/n_;
                m_ *=double(n_)/(n_+1);
            }
            else
                m_ = v;
            n_++;
        }
        double get(){return m_;}
        
    private:
        double m_;
        std::size_t n_;
    };

    int OrbDescDis(const cv::Mat &a, const cv::Mat &b);

	cv::Mat makeP(cv::Mat T, cv::Mat k);
	
    cv::Mat from34To44(cv::Mat T);
	cv::Mat getR(cv::Mat T);
	cv::Mat getT(cv::Mat T);
	cv::Mat computeF(cv::Mat T1, cv::Mat k1
			, cv::Mat T2, cv::Mat k2);

	cv::Mat makeT0();
	cv::Mat makeT(cv::Mat R, cv::Mat t);
	
	cv::Mat makeP0(const cv::Mat k);
	cv::Mat makeP(const cv::Mat& k, const cv::Mat& R, const cv::Mat& t);

	cv::Mat makeP(cv::Mat T, cv::Mat k);


	std::vector<cv::Point2f> to_vector_point2f(
			const std::vector<cv::KeyPoint>& kpts);

	cv::Point3f to_point3f(cv::Mat p);
    cv::Mat p3ftoMat(cv::Point3f p3);
    cv::Mat pf3toMat(cv::Point3f);

    
    std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
    
    g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
    
    cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    cv::Mat toCvMat(const Eigen::Matrix3d &m);
    cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    
    Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    
    std::vector<float> toQuaternion(const cv::Mat &M);
    
    //get the scale factor set
    const std::vector<float>& calcScaleSet(float scale, int nlevel);
    
    //get the square of scale factor set
    const std::vector<float>& calcScaleSquareSet(float scale, int nlevel);
    
    //get the inverse of scale factor set
    const std::vector<float>& calcScaleSetInv(float scale, int nlevel);
    
    //get the inverse square of scale factor set
    const std::vector<float>& calcScaleSquareSetInv(float scale, int nlevel);
    
    cv::Mat getCamCenterFromPose(cv::Mat pose);
    
    inline cv::Mat to_CV_64F(cv::Mat m){
        if(m.empty())
            return m;
        
        if(m.type() != CV_64F){
            cv::Mat _m;
            m.convertTo(_m, CV_64F);
            return _m;
        }
        return m;
    }
    inline cv::Mat to_CV_32F(cv::Mat m)
    {
        if(m.empty())
            return m;
        
        if(m.type() != CV_32F){
            cv::Mat _m;
            m.convertTo(_m, CV_32F);
            return _m;
        }
        
        return m;
    }

}//algo
