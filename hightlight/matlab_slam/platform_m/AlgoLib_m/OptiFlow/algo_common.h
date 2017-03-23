#pragma once
#include <random>
#include <stdint.h>
#include <string>
#include <set>
#include <opencv2/opencv.hpp>

namespace algo
{
    cv::Mat getCameraCenter(cv::Mat p);

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

	
	cv::Mat SkewSymmetry(cv::Mat v);

	cv::Mat makeP(cv::Mat T, cv::Mat k);
	
	cv::Mat getR(cv::Mat T);
	cv::Mat getT(cv::Mat T);
	cv::Mat computeF(cv::Mat T1, cv::Mat k1
			, cv::Mat T2, cv::Mat k2);

	cv::Mat makeT0();
	cv::Mat makeT(cv::Mat R, cv::Mat t);
	
	cv::Mat makeP0(const cv::Mat k);
	cv::Mat makeP(const cv::Mat& k, const cv::Mat& R, const cv::Mat& t);

	cv::Mat makeP(cv::Mat T, cv::Mat k);


	float checkF(cv::Mat F12, const cv::Point2f& x1, const cv::Point2f& x2);
	float checkF(cv::Mat F12, cv::Mat x1, cv::Mat x2);

	int OrbDescDis(const cv::Mat &a, const cv::Mat &b);


	class UniformRandom{
	public:
		UniformRandom(){
		}
		void reset(int min, int max){
			dist_ = std::uniform_int_distribution<>(min,max);
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
		std::mt19937 rng_; 
		std::uniform_int_distribution<> dist_;
	};
	

	std::vector<cv::Point2f> to_vector_point2f(
			const std::vector<cv::KeyPoint>& kpts);

	cv::Point3f to_point3f(cv::Mat p);

	template<typename _T>
	bool eq(_T f1, _T f2) {
		return std::abs(f1 - f2)<std::numeric_limits<_T>::epsilon();
	}
	template<typename _T>
	bool eq(const cv::Point_<_T>& p1, const cv::Point_<_T>& p2){
		return eq(p1.x, p2.x) && eq(p1.y, p2.y);
	}


	double sampsonDistance(cv::Mat F
		,const cv::Point2f& pt1
		, const cv::Point2f& pt2);

	double sampsonDistance(cv::Mat F
			, cv::Mat m1, cv::Mat m2);
    
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
