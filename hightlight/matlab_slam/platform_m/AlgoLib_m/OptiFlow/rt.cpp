#include "rt.h"
#include "algo_common.h"


namespace algo
{
    Triangulate::Triangulate()
	{

	}

	bool Triangulate::operator()(const cv::Point2f& pt1, cv::Mat p1
				, const cv::Point2f& pt2, cv::Mat p2
				, cv::Mat& x3D, float& err)
	{
		_triangulate(pt1, pt2, p1, p2, x3D);
		if(!_checkP3d(x3D, p1, p2, pt1, pt2, err) )
			return false;

		return true;
	}

	void Triangulate::_triangulate(
			const cv::Point2f &kp1, const cv::Point2f &kp2
			, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
	{
		cv::Mat A(4,4,CV_32F);

		A.row(0) = kp1.x*P1.row(2)-P1.row(0);
		A.row(1) = kp1.y*P1.row(2)-P1.row(1);
		A.row(2) = kp2.x*P2.row(2)-P2.row(0);
		A.row(3) = kp2.y*P2.row(2)-P2.row(1);

		cv::Mat u,w,vt;
		cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
		x3D = vt.row(3).t();
		x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
	}

	bool Triangulate::_checkP3d(const cv::Mat& x3D
			, const cv::Mat& P1
			, const cv::Mat& P2
			, const cv::Point2f& pt1
			, const cv::Point2f& pt2
			, float& err)
	{
		if(x3D.at<float>(2)<= 0)
			return false;

		float f1 = x3D.at<float>(0);
		float f2 = x3D.at<float>(1);
		float f3 = x3D.at<float>(2);
		if(std::isinf(f1) || std::isnan(f1)
				|| std::isinf(f2) || std::isnan(f2)
				|| std::isinf(f3) || std::isnan(f3))
		{
			return false;
		}

		cv::Mat X = (cv::Mat_<float>(4,1)
				<<x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2),1
				);
		cv::Mat x1 = P1 *X;
		cv::Mat x2 = P2 *X;
		if(x1.at<float>(2) <=0 || x2.at<float>(2)<=0){
			return false;
		}

		x1/=x1.at<float>(2);
		x2/=x2.at<float>(2);

		
		err = (pt1.x - x1.at<float>(0) ) * (pt1.x - x1.at<float>(0) )
			+ (pt1.y - x1.at<float>(1) ) * (pt1.y - x1.at<float>(1) )
			+ (pt2.x - x2.at<float>(0) ) * (pt2.x - x2.at<float>(0) )
			+ (pt2.y - x2.at<float>(1) ) * (pt2.y - x2.at<float>(1) )
			;
		return true;
	}

	bool FTriangulate::operator()(
			const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, cv::Mat F, cv::Mat k
			, cv::Mat& R, cv::Mat& t
			, std::vector<cv::Mat>& x3Ds
			, std::vector<float>& errs
			, std::vector<bool>& inliners//in_out
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

		std::vector<cv::Point2f> iPts1, iPts2;
		std::vector<std::size_t> idxes;
		for(std::size_t i =0 ;i< inliners.size(); i++){
			if(inliners[i]){
				iPts1.push_back(pts1[i]);
				iPts2.push_back(pts2[i]);
				idxes.push_back(i);
			}
		}
		std::vector<cv::Mat> ix3Ds;
		std::vector<float> iErrs;
		std::vector<bool> iInliners;
		if(!_triangles(iPts1, iPts2, F, k,R,t,ix3Ds, iErrs,iInliners)){
			return false;
		}

		x3Ds.resize(pts1.size());
		errs.resize(pts1.size(), -1);
		for(std::size_t i = 0; i< iInliners.size(); i++){
			if(iInliners[i]){
				x3Ds[idxes[i]] = ix3Ds[i];
				errs[idxes[i]] = iErrs[i];
			}
			else{
				inliners[idxes[i]] = false;
			}
		}

		return true;	
	}


	bool FTriangulate::_triangles(
				const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2
				, cv::Mat F, cv::Mat k
				, cv::Mat& R, cv::Mat& t
				, std::vector<cv::Mat>& x3Ds
				, std::vector<float>& errs
				, std::vector<bool>& inliners)
	{	
		assert(!k.empty());

		cv::Mat E = k.t() * F * k;
		cv::Mat R1,R2;
		_decomposeE(E,R1,R2,t);
		cv::Mat t1 = t;
		cv::Mat t2 = -t;

		std::vector<cv::Mat> x3Ds1, x3Ds2, x3Ds3, x3Ds4;
		std::vector<float> errs1, errs2, errs3, errs4;
		std::vector<bool> inliners1, inliners2, inliners3, inliners4;

		int nGood1 = _checkRT(k, pts1, pts2, R1, t1, x3Ds1, errs1, inliners1);
		int nGood2 = _checkRT(k, pts1, pts2, R2, t1, x3Ds2, errs2, inliners2);
		int nGood3 = _checkRT(k, pts1, pts2, R1, t2, x3Ds3, errs3, inliners3);
		int nGood4 = _checkRT(k, pts1, pts2, R2, t2, x3Ds4, errs4, inliners4);

		std::vector<std::pair<int, int> > goodSort;
		goodSort.push_back(std::make_pair(nGood1, 1));
		goodSort.push_back(std::make_pair(nGood2, 2));
		goodSort.push_back(std::make_pair(nGood3, 3));
		goodSort.push_back(std::make_pair(nGood4, 4));

		std::sort(goodSort.begin(), goodSort.end(), 
				[](const std::pair<int,int>& p1, const std::pair<int,int>& p2)->bool{
					return p1.first > p2.first;
				}
				);

		if(goodSort[0].first < 30){
			return false;
		}

		float similar = float(goodSort[1].first) / goodSort[0].first;
		if(similar> 0.7){
			return false;
		}

		switch(goodSort[0].second){
			case 1:
				R = R1;
				t = t1;
				x3Ds =x3Ds1;
				errs = errs1;
				inliners = inliners1;
				break;

			case 2:
				R = R2;
				t = t1;
				x3Ds =x3Ds2;
				errs = errs2;
				inliners = inliners2;

				break;

			case 3:
				R = R1;
				t = t2;
				x3Ds =x3Ds3;
				errs = errs3;
				inliners = inliners3;

				break;

			case 4:
				R = R2;
				t = t2;
				x3Ds =x3Ds4;
				errs = errs4;
				inliners = inliners4;

				break;

			default:
				assert(0);
		}


		return true;	
	}

	void FTriangulate::_decomposeE(
			const cv::Mat &E
			, cv::Mat &R1, cv::Mat &R2
			, cv::Mat &t)
	{
		cv::Mat u,w,vt;
		cv::SVD::compute(E,w,u,vt);

		u.col(2).copyTo(t);
		t=t/cv::norm(t);

		cv::Mat W(3,3,CV_32F,cv::Scalar(0));
		W.at<float>(0,1)=-1;
		W.at<float>(1,0)=1;
		W.at<float>(2,2)=1;

		R1 = u*W*vt;
		if(cv::determinant(R1)<0)
			R1=-R1;

		R2 = u*W.t()*vt;
		if(cv::determinant(R2)<0)
			R2=-R2;
	}

	int FTriangulate::_checkRT(const cv::Mat& k
			, const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, const cv::Mat& R, const cv::Mat& t
			, std::vector<cv::Mat>& x3Ds
			, std::vector<float>& errs
			, std::vector<bool>& inliners)
	{
		cv::Mat p0= makeP0(k);
		cv::Mat p1 = makeP(k, R, t);

		int nInliners = 0;
		Triangulate tri;
		for(std::size_t i =0; i< pts1.size(); i++)
		{
			cv::Mat x3D;
			float err;
			if( tri(pts1[i], p0, pts2[i], p1, x3D, err) ){
				x3Ds.push_back(x3D);
				errs.push_back(err);	
				inliners.push_back(true);
				nInliners++;
			}else{
				x3Ds.push_back(cv::Mat());
				errs.push_back(-1);
				inliners.push_back(false);
			}	
		}
		return nInliners;
	}

	FHTrans::FHTrans()
	{
		nIter_ = 200;
		sigma_ = 1.0;
	}

	void FHTrans::setPts(const std::vector<cv::Point2f>& pts1
				, const std::vector<cv::Point2f>& pts2)
	{
		pts1_ = pts1;
		pts2_ = pts2;	
	}
	void FHTrans::setInliners(const std::vector<bool>& inliners)
	{
		inliners_ = inliners;
	}

	bool FHTrans::operator()()
	{
		F_ = cv::Mat();
		fInliners_.clear();
		//-----------------------

		const int min_pts = 40;
		if(pts1_.size() != pts2_.size()){
			return false;
		}
		if(pts1_.size() != inliners_.size()){
			if(!inliners_.empty()){
				return false;
			}
			inliners_.resize(pts1_.size(), true);
		}

		std::vector<std::size_t> idxes;
		std::vector<cv::Point2f> pts1, pts2;
		std::vector<bool> fInliners;
		for(std::size_t i = 0; i< inliners_.size(); i++){
			if(inliners_[i]){
				idxes.push_back(i);
				pts1.push_back(pts1_[i]);
				pts2.push_back(pts2_[i]);
			}
		}
		
		if(pts1.size() < min_pts){
			return false;
		}

		_resetRansac(pts1.size(), nIter_, 8, ransacIdx_);

		float score = 0;
		_computeF(pts1, pts2, F_, fInliners, score);
		fInliners_ = inliners_;
		for(std::size_t i = 0; i< fInliners.size(); i++){
			if(!fInliners[i]){
				fInliners_[idxes[i]] = false;
			}
		}

		return true;
	}

	std::vector<bool> FHTrans::getFInliners()
	{
		return fInliners_;
	}

	cv::Mat FHTrans::getF()
	{
		return F_;
	}

	void FHTrans::_computeF(	const std::vector<cv::Point2f>& _pts1
				, const std::vector<cv::Point2f>& _pts2
				, cv::Mat& F
				, std::vector<bool>& inliners
				, float& score)
	{
		std::vector<cv::Point2f> pts1 = _pts1;	
		std::vector<cv::Point2f> pts2 = _pts2;

		cv::Mat t1 = _normalize(pts1);
		cv::Mat t2 = _normalize(pts2);
		cv::Mat t2t = t2.t();

		score = 0.0;

		for(std::size_t i = 0; i< ransacIdx_.size(); i++)
		{
			std::vector<cv::Point2f> racP1(ransacIdx_[i].size());
			std::vector<cv::Point2f> racP2(ransacIdx_[i].size());
			for(std::size_t j = 0; j< ransacIdx_[i].size(); j++)
			{
				int idx = ransacIdx_[i][j];
				
				racP1[j] = pts1[idx];
				racP2[j] = pts2[idx];
			}

			cv::Mat tmpF = __computeF(racP1, racP2);
			tmpF = t2t*tmpF*t1;
			
			std::vector<bool> tmpInlines;
			float tmpScore = _checkF(_pts1, _pts2,tmpF, tmpInlines, sigma_);

			if(tmpScore > score){
				score = tmpScore;
				std::swap(tmpF, F);
				std::swap(tmpInlines, inliners);
			}
		}
	}


	cv::Mat FHTrans::_normalize(std::vector<cv::Point2f>& pts)
	{
		float meanX = 0;
		float meanY = 0;

		std::vector<cv::Point2f> rawPts;
		std::swap(pts, rawPts);
		pts.resize(rawPts.size());


		int N = rawPts.size();
		for(int i=0; i< (int)rawPts.size(); i++)
		{
			meanX += rawPts[i].x;
			meanY += rawPts[i].y;
		}

		meanX = meanX/N;
		meanY = meanY/N;

		float meanDevX = 0;
		float meanDevY = 0;

		for(int i=0; i<N; i++)
		{
			pts[i].x = rawPts[i].x - meanX;
			pts[i].y = rawPts[i].y - meanY;

			meanDevX += std::abs(pts[i].x);
			meanDevY += std::abs(pts[i].y);
		}

		meanDevX = meanDevX/N;
		meanDevY = meanDevY/N;

		float sX = 1.0/meanDevX;
		float sY = 1.0/meanDevY;

		for(int i=0; i<N; i++)
		{
			pts[i].x = pts[i].x * sX;
			pts[i].y = pts[i].y * sY;
		}

		cv::Mat T = cv::Mat::eye(3,3,CV_32F);
		T.at<float>(0,0) = sX;
		T.at<float>(1,1) = sY;
		T.at<float>(0,2) = -meanX*sX;
		T.at<float>(1,2) = -meanY*sY;
		return T;
	}

	cv::Mat FHTrans::__computeF(const std::vector<cv::Point2f> &vP1
		,const std::vector<cv::Point2f> &vP2)
	{

		const int N = vP1.size();

		cv::Mat A(N,9,CV_32F);

		for(int i=0; i<N; i++)
		{
			const float u1 = vP1[i].x;
			const float v1 = vP1[i].y;
			const float u2 = vP2[i].x;
			const float v2 = vP2[i].y;

			A.at<float>(i,0) = u2*u1;
			A.at<float>(i,1) = u2*v1;
			A.at<float>(i,2) = u2;
			A.at<float>(i,3) = v2*u1;
			A.at<float>(i,4) = v2*v1;
			A.at<float>(i,5) = v2;
			A.at<float>(i,6) = u1;
			A.at<float>(i,7) = v1;
			A.at<float>(i,8) = 1;
		}

		cv::Mat u,w,vt;

		cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

		cv::Mat Fpre = vt.row(8).reshape(0, 3);

		cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

		w.at<float>(2)=0;

		return  u*cv::Mat::diag(w)*vt;
	}

	void FHTrans::_resetRansac(
			int nSample, int nIter, int nPoint
			, std::vector<std::vector<std::size_t> >& idx)
	{
		UniformRandom random;
		random.reset(0, nSample-1);

		idx = std::vector<std::vector<std::size_t> >(nIter, std::vector<std::size_t>(nPoint));
		for(int i = 0; i< nIter; i++){
			std::vector<int> vs = random.get_unique(nPoint);
			for(int j = 0; j< nPoint; j++)
			{
				idx[i][j] = vs[j];
			}	
		}
	}

	float FHTrans::_checkF(
			const std::vector<cv::Point2f>& pts1
			, const std::vector<cv::Point2f>& pts2
			, const cv::Mat &F21
			, std::vector<bool> &vbMatchesInliers
			, float sigma)
	{
		const int N = pts1.size();

		const float f11 = F21.at<float>(0,0);
		const float f12 = F21.at<float>(0,1);
		const float f13 = F21.at<float>(0,2);
		const float f21 = F21.at<float>(1,0);
		const float f22 = F21.at<float>(1,1);
		const float f23 = F21.at<float>(1,2);
		const float f31 = F21.at<float>(2,0);
		const float f32 = F21.at<float>(2,1);
		const float f33 = F21.at<float>(2,2);

		vbMatchesInliers.resize(N);

		float score = 0;

		const float th = 3.841;
		const float thScore = 5.991;

		const float invSigmaSquare = 1.0/(sigma*sigma);

		for(int i=0; i<N; i++)
		{
			bool bIn = true;

			const cv::Point& p1 = pts1[i];
			const cv::Point& p2 = pts2[i];

			const float u1 = p1.x;
			const float v1 = p1.y;
			const float u2 = p2.x;
			const float v2 = p2.y;

			// Reprojection error in second image
			// l2=F21x1=(a2,b2,c2)

			const float a2 = f11*u1+f12*v1+f13;
			const float b2 = f21*u1+f22*v1+f23;
			const float c2 = f31*u1+f32*v1+f33;

			const float num2 = a2*u2+b2*v2+c2;

			const float squareDist1 = num2*num2/(a2*a2+b2*b2);

			const float chiSquare1 = squareDist1*invSigmaSquare;

			if(chiSquare1>th)
				bIn = false;
			else
				score += thScore - chiSquare1;

			// Reprojection error in second image
			// l1 =x2tF21=(a1,b1,c1)

			const float a1 = f11*u2+f21*v2+f31;
			const float b1 = f12*u2+f22*v2+f32;
			const float c1 = f13*u2+f23*v2+f33;

			const float num1 = a1*u1+b1*v1+c1;

			const float squareDist2 = num1*num1/(a1*a1+b1*b1);

			const float chiSquare2 = squareDist2*invSigmaSquare;

			if(chiSquare2>th)
				bIn = false;
			else
				score += thScore - chiSquare2;

			if(bIn)
				vbMatchesInliers[i]=true;
			else
				vbMatchesInliers[i]=false;
		}

		return score;
	}
}//
