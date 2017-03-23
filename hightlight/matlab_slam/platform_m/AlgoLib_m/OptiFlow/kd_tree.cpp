#include "kd_tree.h"


namespace algo
{
	void KdSearch_P2f::init(const std::vector<cv::Point2f>& pts)
	{
		pts_.pts = pts;
		kd_.reset(new kd_tree::KdTree_P2f(
					2, pts_, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
		kd_->buildIndex();	
	}

	cv::Point2f KdSearch_P2f::getNearest(const cv::Point2f _q)
	{
		float err = 0;
		return getNearest(_q, err);
	}

	cv::Point2f KdSearch_P2f::getNearest(const cv::Point2f _q, float& err)
	{
		float q[2]={_q.x, _q.y}; 

		const size_t num_results = 1;
		size_t ret_index;
		nanoflann::KNNResultSet<float> resultSet(num_results);
		resultSet.init(&ret_index, &err );
		kd_->findNeighbors(resultSet, &q[0], nanoflann::SearchParams(10));

		return pts_.pts[ret_index];
	
	}
	
	
	void WndSearch_P2f::init(const std::vector<cv::Point2f>& pts)
	{
        
        for(std::size_t i = 0; i< pts.size(); i++){
            int x = cvRound(pts[i].x);
            int y = cvRound(pts[i].y);
            row_.insert(x);
            col_.insert(y);
            
            std::pair<int,int> key(x, y);
            pos2idx_[key] = i;
        }
        
//		for(std::size_t i = 0; i< pts.size(); i++){
//			int x = cvRound(pts[i].x);
//			int y = cvRound(pts[i].y);
//			row_.insert(x);
//			col_.insert(y);	
//
//			std::pair<int,int> key(x, y);
//			pos2idx_[i] = key;
//		}
        
	}

	std::set<std::size_t> WndSearch_P2f::getIdxInArea(const cv::Rect& r) const
	{
        std::set<std::size_t> idx;
        for(auto it = row_.lower_bound(r.x)
            ; it!=row_.upper_bound(r.x+ r.width); it++)
        {
            for(auto jt = col_.lower_bound(r.y)
                ; jt!=col_.upper_bound(r.y+ r.height); jt++)
            {
                std::pair<int, int> key(*it, *jt);
                auto val = pos2idx_.find(key);
                if(val!= pos2idx_.end()){
                    idx.insert(val->second);
                }
            }
        }
        return idx;
        
        
//		std::set<std::size_t> idx;
//		for(auto it = row_.lower_bound(r.x)
//				; it!=row_.upper_bound(r.x+ r.width); it++)
//		{
//			for(auto jt = col_.lower_bound(r.y)
//					; jt!=col_.upper_bound(r.y+ r.height); jt++)
//			{
//				std::pair<int, int> key(*it, *jt);
//                for( const auto& n : pos2idx_ ) {
//                    if(n.second == key){
//                        idx.insert(n.first);
//                    }
//                }
//			}
//		}
//		return idx;
	}

	std::set<std::size_t> WndSearch_P2f::getIdxInArea(
		const cv::Point& pt, const cv::Size& s) const
	{
		cv::Rect r;
		r.x = pt.x - s.width/2;
		r.y = pt.y - s.height/2;
		r.width = s.width;
		r.height = s.height;

		return getIdxInArea(r);
	}


}//
