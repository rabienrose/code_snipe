#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

namespace algo
{
    class Triangulate
	{
	public:
		Triangulate();

		bool operator()(const cv::Point2f& pt1, cv::Mat p1
				, const cv::Point2f& pt2, cv::Mat p2
				, cv::Mat& x3D ,float& err
				);
	
	private:
		void _triangulate(
			const cv::Point2f &kp1, const cv::Point2f &kp2
			, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

		bool _checkP3d(const cv::Mat& x3D
				, const cv::Mat& P1
				, const cv::Mat& P2
				, const cv::Point2f& pt1
				, const cv::Point2f& pt2
				, float& err);
	};	

	class FTriangulate
	{
	public:
		bool operator()(
				const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2
				, cv::Mat F, cv::Mat k
				, cv::Mat& R, cv::Mat& t
				, std::vector<cv::Mat>& x3Ds
				, std::vector<float>& errs
				, std::vector<bool>& inliners//in_out
				);

	private:
		bool _triangles(const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2
				, cv::Mat F, cv::Mat k
				, cv::Mat& R, cv::Mat& t
				, std::vector<cv::Mat>& x3Ds
				, std::vector<float>& errs
				, std::vector<bool>& inliners);

		void _decomposeE(
			const cv::Mat &E
			, cv::Mat &R1, cv::Mat &R2
			, cv::Mat &t);

		int _checkRT(const cv::Mat& k
			, const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, const cv::Mat& R, const cv::Mat& t
			, std::vector<cv::Mat>& x3Ds
			, std::vector<float>& errs
			, std::vector<bool>& inliers);
	};
    
	class FHTrans
	{
	public:
		FHTrans();

		void setPts(const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2);
		void setInliners(const std::vector<bool>& inliners);

		bool operator()();

		std::vector<bool> getFInliners();
		cv::Mat getF();

	private:
		void _computeF(	const std::vector<cv::Point2f>& _pts1
				, const std::vector<cv::Point2f>& _pts2
				, cv::Mat& F
				, std::vector<bool>& inliners
				, float& score);

		cv::Mat __computeF(const std::vector<cv::Point2f> &vP1
			,const std::vector<cv::Point2f> &vP2);
		cv::Mat _normalize(std::vector<cv::Point2f>& pts);

		void _resetRansac(
			int nSample, int nIter, int nPoint
			, std::vector<std::vector<std::size_t> >& idx);

		float _checkF(const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, const cv::Mat &F21
			, std::vector<bool> &vbMatchesInliers
			, float sigma);

	private:
		std::vector<cv::Point2f> pts1_;
		std::vector<cv::Point2f> pts2_;
		std::vector<bool> inliners_;

		cv::Mat F_;
		std::vector<bool> fInliners_;

		std::vector<std::vector<std::size_t> > ransacIdx_;

		float sigma_;
		int nIter_;
	};
	


}//
