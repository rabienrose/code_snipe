#include "optflowIni.h"
#include "rt.h"
#include <memory>


namespace algo
{
    std::vector<cv::Point2f> FlowMatch::getCorners(cv::Mat gray)
	{
		const int MAX_COUNT = 4000;

		std::vector<cv::Point2f> pts;
		cv::goodFeaturesToTrack(gray, pts, MAX_COUNT, 0.01, 10);

		const cv::Size subPixSize(10,10);
		cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
		cornerSubPix(gray, pts, subPixSize, cv::Size(-1,-1), termcrit);
		return pts;
	}
	

	void FlowMatch::match(cv::Mat preGray
				, const std::vector<cv::Point2f>& prePts
				, cv::Mat curGray
				, std::vector<cv::Point2f>& curPts
				, std::vector<float>& err)
	{
		std::vector<cv::Point2f> tmpPrePts;
		std::vector<std::size_t> tmpPreIdx;
		for(std::size_t  i = 0; i< prePts.size(); i++)
		{
			if(prePts[i].x < 0 || prePts[i].y<0)
				continue;

			tmpPrePts.push_back(prePts[i]);
			tmpPreIdx.push_back(i);
		}

		std::vector<cv::Point2f> tmpCurPts;
		std::vector<float> tmpErr;
		_match(preGray, tmpPrePts, curGray, tmpCurPts, tmpErr);

		curPts  =std::vector<cv::Point2f>(prePts.size(), cv::Point2f(-1,-1));
		err = std::vector<float>(prePts.size(), 0);
		for(std::size_t i = 0; i< tmpPreIdx.size(); i++)
		{
			curPts[tmpPreIdx[i]] = tmpCurPts[i];
			err[tmpPreIdx[i]] = tmpErr[i];
		}
	}
	
	void FlowMatch::match(cv::Mat img1, cv::Mat img2
				, std::vector<cv::Point2f>& p1
				, std::vector<cv::Point2f>& p2
				, std::vector<float>& err)
	{
		std::vector<cv::Point2f> rawP1 = getCorners(img1);
		std::vector<cv::Point2f> rawP2;
		std::vector<float> rawErr;
		_match(img1, rawP1, img2, rawP2, rawErr);

		assert(rawP1.size() == rawP2.size());
		assert(rawP1.size() == rawErr.size());
        
		for(std::size_t i = 0; i< rawP2.size(); i++)
		{
			if(rawP2[i].x< 0 || rawP2[i].y < 0)
				continue;

			p1.push_back(rawP1[i]);
			p2.push_back(rawP2[i]);
			err.push_back(rawErr[i]);
		}
	}	
	
	void FlowMatch::_match(cv::Mat preGray
			, const std::vector<cv::Point2f>& prePts
			,cv::Mat curGray
			, std::vector<cv::Point2f>& curPts
			, std::vector<float>& err)
	{
		std::vector<uchar> status;	
		cv::calcOpticalFlowPyrLK(preGray, curGray, prePts, curPts,status, err);

		for(std::size_t i = 0 ;i< status.size() ;i++)
		{
			if(status[i] == 0)
			{
				curPts[i] = cv::Point2f(-1,-1);
			}
		}
	}

    
    
    OptFlow::OptFlow(cv::Mat img)
	{
		_init(img, flow_math_.getCorners(img));
	}

	OptFlow::OptFlow(cv::Mat img
		, const std::vector<cv::Point2f>& iniPts)
	{
		_init(img, iniPts);
	}
	
	bool OptFlow::operator()(cv::Mat img)
	{
		std::vector<float> err;
		flow_math_.match(preImg_, prePts_
				, img, curPts_,err);

		preImg_ = img;
		prePts_ = curPts_;

		return true;
	}

	void OptFlow::getFlow(std::vector<cv::Point2f>& iniPts
			, std::vector<cv::Point2f>& curPts
			, std::vector<bool>& inliners) const
	{
		iniPts = iniPts_;
		curPts = curPts_;	
		inliners.resize(iniPts.size(), false);
		for(std::size_t i =0; i< iniPts.size(); i++){
			if(iniPts[i].x< 0 || iniPts[i].y < 0
					|| curPts[i].x < 0 || curPts[i].y < 0)
				continue;
			inliners[i] = true;
		}
	}

	void OptFlow::getValidFlow(std::vector<cv::Point2f>& validIniPts
			, std::vector<cv::Point2f>& validCurPts) const
	{
		for(std::size_t i =0; i< iniPts_.size(); i++){
			if(iniPts_[i].x< 0 || iniPts_[i].y < 0
					|| curPts_[i].x < 0 || curPts_[i].y < 0)
				continue;
			validIniPts.push_back(iniPts_[i]);
			validCurPts.push_back(curPts_[i]);
		}
	}

	void OptFlow::_init(cv::Mat img
		, const std::vector<cv::Point2f>& iniPts)
	{
		preImg_ = img;
		iniPts_=  iniPts;
		prePts_ = iniPts;
	}

    
    
    
    N2Filter::N2Filter(float v)
	{
		v2_ = v*v;	
	}

	bool N2Filter::operator()(
		const std::vector<cv::Point2f>& pts1
		, const std::vector<cv::Point2f>& pts2
		, std::vector<cv::Point2f>& validPts1
		, std::vector<cv::Point2f>& validPts2)
	{
		if(pts1.size() != pts2.size()){
			return false;
		}

		std::vector<cv::Point2f> fPts1, fPts2;
		for(std::size_t i = 0; i< pts1.size(); i++){
			if(_check_n2(pts1[i], pts2[i]) ){
				fPts1.push_back(pts1[i]);
				fPts2.push_back(pts2[i]);
			}
		}

		std::swap(validPts1, fPts1);
		std::swap(validPts2, fPts2);

		return true;
	}

	bool N2Filter::operator()(
			const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, std::vector<bool>& inliners //in_out
			)
	{
		if(pts1.size() != pts2.size()){
			return false;
		}
		if(pts1.size() != inliners.size()){
			if(!inliners.empty()){
				return false;
			}
			inliners.resize(pts1.size(), true);
		}
		for(std::size_t i = 0; i< pts1.size(); i++){
			if(inliners[i]){
				if(!_check_n2(pts1[i], pts2[i]) ){
					inliners[i] = false;
				}
			}
		}

		return true;
	}
	
	bool N2Filter::_check_n2(const cv::Point2f& p1, const cv::Point2f& p2)
	{
		return _n2(p1,p2) > v2_;	
	}

	float N2Filter::_n2(const cv::Point2f& p1, const cv::Point2f& p2)
	{
		float d2 = (p1.x - p2.x) * (p1.x - p2.x)
			+ (p1.y - p2.y) * (p1.y - p2.y);
		return d2;
	}
	
	OrbWindowMatchFilter::OrbWindowMatchFilter(const cv::Size wndSize)
	{
		wndSize_ = wndSize;
	}

	bool OrbWindowMatchFilter::operator()(
			cv::Mat matchDesc1
			, const std::vector<cv::Point2f>& matchPts2
			, std::vector<cv::KeyPoint>& kpts2
			, cv::Mat desc2
			, std::vector<bool>& inliners)
	{
		matches_.clear();
		//
		
		if(matchDesc1.rows != (int)matchPts2.size()){
			return false;
		}
		if(matchPts2.size() != inliners.size()){
			if(!inliners.empty()){
				return false;
			}
			inliners.resize(matchPts2.size(), true);
		}

		WndSearch_P2f wnd_search2;
		wnd_search2.init(algo::to_vector_point2f(kpts2));

		for(std::size_t i = 0; i< matchPts2.size(); i++){
			if(inliners[i]){
				std::size_t match2Idx=0;
				if(!_check(matchDesc1.row(i), matchPts2[i]
						, kpts2, desc2, wnd_search2, match2Idx))
				{
					inliners[i] = false;
				}
				else{
					matches_.push_back(std::make_pair(i, match2Idx));
				}
			}
		}

		return true;
	}

	bool OrbWindowMatchFilter::_check(cv::Mat descRow1
			, cv::Point2f pt2
			, std::vector<cv::KeyPoint>& kpts2
			, cv::Mat desc2
			,  WndSearch_P2f& window_search2
			, std::size_t& match2Idx)
	{

		const int threshold = 500;
		const float dis2_threshold = 9.1;

		std::set<std::size_t> idxes = window_search2.getIdxInArea(
				pt2,wndSize_);

		algo::Min2Compare<int> min2(threshold);
		for(auto it = idxes.begin(); it!=idxes.end(); it++){
			cv::Mat descRow2 = desc2.row(*it);
			int dis = OrbDescDis(descRow1, descRow2);
			min2(dis, *it);
		}
		if(min2.getMin1() >= threshold)
			return false;
		if( (float)min2.getMin1() >= 0.8 * min2.getMin2() )
			return false;


		int minIdx = min2.getMinIdx1();
		if(minIdx == -1)
			return false;
		cv::Point orb_pt2 = kpts2[minIdx].pt;
		float dis2 = (orb_pt2.x - pt2.x) * 	(orb_pt2.x - pt2.x)
			+(orb_pt2.y - pt2.y) *(orb_pt2.y - pt2.y);
		if(dis2 > dis2_threshold)
			return false;

		match2Idx = minIdx;
		return true;
	}
	
	OptflowSelector::OptflowSelector(cv::Mat k)
	{
		k_ = k;
	}

	void OptflowSelector::operator()(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc)
	{
		for(auto it = flows_.begin(); it!=flows_.end(); it++){
			_extendFlow(*it, img, kps, desc);
		}
        
		_chooseBest();
		if(_needNewFlow()){
			auto flow = _initFlow(img, kps, desc);
			flows_.push_back(flow);
		}
		_delFlow();
	}

	void OptflowSelector::reset()
	{
		bestFlow_.reset();
		flows_.clear();
	}

	std::shared_ptr<OptflowSelector::OptflowInfo> 
		OptflowSelector::getBestOptflow()	
	{
		return bestFlow_;
	}

	std::shared_ptr<OptflowSelector::OptflowInfo> 
		OptflowSelector::_initFlow(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc )
	{
		std::shared_ptr<OptflowSelector::OptflowInfo> flow = std::make_shared<OptflowInfo>();
		flow->del_flag = false;
		flow->img1 = img;
        flow->kpts1 = kps;
        flow->desc1 = desc;
		flow->pts1 = algo::to_vector_point2f(flow->kpts1);
		flow->opt_flow.reset(new algo::OptFlow(img, flow->pts1));
		flow->nOrb3Dinlines = 0;

		return flow;
	}

	bool OptflowSelector::_needNewFlow()
	{
		if(flows_.empty())
			return true;

		auto flow = flows_.back();

		if(flow->rt_inlines.empty()){
			flow->del_flag = true;
			return true;
		}
		
		float inlineRate = _getInlineRate(flow->rt_inlines);
		if(inlineRate < 0.9)
			return true;

		return false;
	}

	void OptflowSelector::_delFlow()
	{
		while(true){
			bool do_del = false;

			for(auto it = flows_.begin(); it!=flows_.end(); it++){
				if(!*it 
						|| (*it)->del_flag
						)
				{
					do_del = true;
					flows_.erase(it);
					break;
				}
			}

			if(!do_del)
				break;
		}
	}

	void OptflowSelector::_extendFlow(std::shared_ptr<OptflowInfo> flow
			, cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc)
	{
		flow->img2 = img;
		(*flow->opt_flow)(img);
		flow->opt_flow->getFlow(flow->pts1, flow->pts2, flow->rt_inlines);

		algo::FHTrans fh_trans;
		fh_trans.setPts(flow->pts1, flow->pts2);
		fh_trans.setInliners(flow->rt_inlines);
		if(!fh_trans() ){
			flow->del_flag = true;
			return;
		}
		flow->rt_inlines = fh_trans.getFInliners();
		cv::Mat F = fh_trans.getF();

		float inlineRate = _getInlineRate(flow->rt_inlines);
		if(inlineRate< 0.7){
			flow->del_flag = true;
			return;
		}

		flow->inlines = flow->rt_inlines;
		//algo::N2Filter n2_filter(4);
		//n2_filter(flow->pts1, flow->pts2, flow->inlines);

		flow->rawX3Ds.clear();
		flow->x3Ds.clear();
		cv::Mat R,t;
		std::vector<float> errs;
		algo::FTriangulate f_tri;
		if(!f_tri(flow->pts1, flow->pts2,F,k_
					,flow->R,flow->t,flow->rawX3Ds
					, errs, flow->inlines))
		{
			flow->del_flag = true;
			return;
		}
        
        flow->kpts2= kps;
        flow->desc2 = desc;

		algo::OrbWindowMatchFilter orb_wnd_filter(cv::Size(100,100));
		if(!orb_wnd_filter(flow->desc1, flow->pts2
				, flow->kpts2, flow->desc2, flow->inlines) )
		{
			flow->del_flag = true;
			return;
		}
		flow->matches = orb_wnd_filter.getMatches();
		for(auto it = flow->matches.begin(); it!=flow->matches.end(); it++)
		{
			flow->x3Ds.push_back(flow->rawX3Ds[it->first]);	
		}

		flow->nOrb3Dinlines = 0;
		for(auto it = flow->inlines.begin(); it!=flow->inlines.end(); it++){
			if(*it){
				flow->nOrb3Dinlines++;
			}
		}
	}

	float OptflowSelector::_getInlineRate(const std::vector<bool>& inlines)
	{
		if(inlines.empty())
			return 0;

		int nInline = 0;
		for(auto it = inlines.begin(); it!=inlines.end(); it++){
			if(*it){
				nInline++;
			}
		}
		float inlineRate = float(nInline)/ inlines.size();
		return inlineRate;
	}
	
	void OptflowSelector::_chooseBest()
	{
		const int threshold = 200;
		bestFlow_.reset();
        
		for(auto it = flows_.begin(); it!=flows_.end(); it++){
            if( (*it)->nOrb3Dinlines > threshold){
				if(!*it)
					continue;

				if( (*it)->del_flag )
					continue;

				if(!bestFlow_){
					bestFlow_ = *it;
				}
				else if( (*it)->nOrb3Dinlines > bestFlow_->nOrb3Dinlines ){
					bestFlow_ = *it;
				}
			}	
		}
	}



	OptflowIni::OptflowIni()
	{
		n_ = 0;
	}

	bool OptflowIni::isInit()
	{
		return (bool)flow_selector_;
	}
	void OptflowIni::init(cv::Mat k)
	{
		flow_selector_.reset(new OptflowSelector(k));	
	}


	void OptflowIni::operator()(cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat desc)
	{
		n_++;
		if(flow_selector_){
			(*flow_selector_)(img, kps, desc);
		}	
		else{
			assert(0);
		}
	}
	void OptflowIni::reset()
	{
		flow_selector_->reset();
	}

	bool OptflowIni::getIniRet( std::vector<std::pair<std::size_t, std::size_t> >& matches
                , std::vector<cv::Mat>& x3Ds
                , cv::Mat& R
                , cv::Mat& t)
	{
		std::shared_ptr<algo::OptflowSelector::OptflowInfo> best = 
			flow_selector_->getBestOptflow();
		if(!best)
			return false;
		matches = best->matches;
        x3Ds = best->x3Ds;

		R = best->R;
		t = best->t;
		return true;	
	}

}//
