#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "kd_tree.h"
#include "algo_common.h"

namespace algo
{
    class FlowMatch
	{
	public:
		std::vector<cv::Point2f> getCorners(cv::Mat gray);	

		//if pts match failed, the pos is <-1,-1>
		void match(cv::Mat preGray
				, const std::vector<cv::Point2f>& prePts
				, cv::Mat curGray
				, std::vector<cv::Point2f>& curPts
				, std::vector<float>& err);
		
		//if pts match failed, the pos is <-1,-1>
		void match(cv::Mat img1, cv::Mat img2
				, std::vector<cv::Point2f>& p1
				, std::vector<cv::Point2f>& p2
				, std::vector<float>& err);

	private:
		void _match(cv::Mat preGray
				, const std::vector<cv::Point2f>& prePts
				, cv::Mat curGray
				, std::vector<cv::Point2f>& curPts
				, std::vector<float>& err);

	};
    
    
    class OptFlow
	{
	public:
		OptFlow(cv::Mat img);
		OptFlow(cv::Mat img
				, const std::vector<cv::Point2f>& iniPts);
		
		bool operator()(cv::Mat img);

		void getFlow(std::vector<cv::Point2f>& iniPts
				, std::vector<cv::Point2f>& curPts
				, std::vector<bool>& inliners) const;
		void getValidFlow(std::vector<cv::Point2f>& validIniPts
				, std::vector<cv::Point2f>& validCurPts) const;

	private:
		void _init(cv::Mat img
				, const std::vector<cv::Point2f>& iniPts);

	private:
		cv::Mat preImg_;
		std::vector<cv::Point2f> iniPts_;
		std::vector<cv::Point2f> prePts_;
		std::vector<cv::Point2f> curPts_;

		FlowMatch flow_math_;
	};
    
    class N2Filter
	{
	public:
		N2Filter(float v);

		bool operator()(
			const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, std::vector<cv::Point2f>& validPts1
			, std::vector<cv::Point2f>& validPts2);

		bool operator()(
				const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2
				, std::vector<bool>& inliners //in_out
				);

	private:
		bool _check_n2(const cv::Point2f& p1, const cv::Point2f& p2);
		float _n2(const cv::Point2f& p1, const cv::Point2f& p2);

	private:
		float v2_;
	};
	


	class OrbWindowMatchFilter
	{
	public:
		OrbWindowMatchFilter(const cv::Size wndSize);

		bool operator()(
				cv::Mat matchDesc1
				, const std::vector<cv::Point2f>& matchPts2
				, std::vector<cv::KeyPoint>& kpts2
				, cv::Mat des2
				, std::vector<bool>& inliners //in_out
				);
		std::vector<std::pair<std::size_t, std::size_t> > getMatches(){return matches_;}

	protected:
		bool _check(cv::Mat descRow1
				, cv::Point2f pt2
				, std::vector<cv::KeyPoint>& kpts2
				, cv::Mat desc2
				,  algo::WndSearch_P2f& window_search2
				, std::size_t& match2Idx);
		
	
	private:
		cv::Size wndSize_;
        std::vector<std::pair<std::size_t, std::size_t> > matches1_;
		std::vector<std::pair<std::size_t, std::size_t> > matches_;
	};

	class OptflowSelector
	{
	public:
		struct OptflowInfo
		{
			cv::Mat img1;
			cv::Mat img2;

			std::vector<cv::KeyPoint> kpts1;
			std::vector<cv::KeyPoint> kpts2;

			cv::Mat desc1;
			cv::Mat desc2;

			std::vector<cv::Point2f> pts1;
			std::vector<cv::Point2f> pts2;

			cv::Mat R,t;
			std::vector<cv::Mat> rawX3Ds;
						
			std::shared_ptr<algo::OptFlow> opt_flow;
			std::vector<bool> rt_inlines;
			std::vector<bool> inlines;

			std::vector<std::pair<std::size_t, std::size_t> > matches;
			std::vector<cv::Mat> x3Ds;

			int nOrb3Dinlines;

			bool del_flag;
		};

	public:
		OptflowSelector(cv::Mat k);

		void operator()(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc);
		void reset();

		std::shared_ptr<OptflowInfo> getBestOptflow();

	private:
		std::shared_ptr<OptflowInfo> _initFlow(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc);
		bool _needNewFlow();
		void _delFlow();
		void _extendFlow(std::shared_ptr<OptflowInfo> flow
				, cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc);

		float _getInlineRate(const std::vector<bool>& inlines);
		void _chooseBest();

	private:
		std::deque<std::shared_ptr<OptflowInfo> > flows_;
		std::shared_ptr<OptflowInfo> bestFlow_;
		cv::Mat k_;
	};	




	class OptflowIni
	{
	public:
		OptflowIni();

		bool isInit();
		void init(cv::Mat k);
		void operator()(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc);
		void reset();

		bool getIniRet( std::vector<std::pair<std::size_t, std::size_t> >& matches
                , std::vector<cv::Mat>& x3Ds
                , cv::Mat& R
                , cv::Mat& t);

	private:
		std::shared_ptr<OptflowSelector> flow_selector_;
		int n_;
		
	};

}//
