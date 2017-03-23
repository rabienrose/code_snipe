#pragma once
#include <set>
#include <opencv/cv.h>
#include <map>
#include "kd_tree/nanoflann.hpp"

namespace algo
{

	namespace kd_tree{
		struct Cloud_P2f{
			std::vector<cv::Point2f> pts;	
			inline size_t kdtree_get_point_count() const { return pts.size(); }

			inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t /*size*/) const
			{
				const float d0=p1[0]-pts[idx_p2].x;
				const float d1=p1[1]-pts[idx_p2].y;
				return d0*d0+d1*d1;
			}

			inline float kdtree_get_pt(const size_t idx, int dim) const
			{
				if (dim==0) return pts[idx].x;
				else return pts[idx].y;
			}

			template <class BBOX>
			bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
		};

		typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<float, Cloud_P2f> ,
			Cloud_P2f,
			2
			> KdTree_P2f;

	}//


	class KdSearch_P2f
	{
	public:
		void init(const std::vector<cv::Point2f>& pts);

		cv::Point2f getNearest(const cv::Point2f q);
		cv::Point2f getNearest(const cv::Point2f q, float& err);

	private:
		std::shared_ptr<kd_tree::KdTree_P2f> kd_;
		kd_tree::Cloud_P2f pts_;
	};


	class WndSearch_P2f
	{
	public:
		void init(const std::vector<cv::Point2f>& pts);

		std::set<std::size_t> getIdxInArea(const cv::Rect& r) const;
		std::set<std::size_t> getIdxInArea(
			const cv::Point& pt, const cv::Size& s) const;

	private:
		std::set<int> row_;
		std::set<int> col_;
		std::map<std::pair<int,int>, std::size_t > pos2idx_;

	};
}//
