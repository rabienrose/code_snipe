#include "algo_common.h"

namespace{
	template<typename _T>
	cv::Mat SkewSymmetry_Impl(cv::Mat v)
	{
		cv::Mat m =  (cv::Mat_<_T>(3,3) <<
			0, -v.at<_T>(2), v.at<_T>(1),
            v.at<_T>(2), 0, -v.at<_T>(0),
            -v.at<_T>(1), v.at<_T>(0), 0);

		return m;
	}
}

namespace algo
{
    cv::Mat getCameraCenter(cv::Mat p)
    {
        assert(p.cols == 4 && p.rows == 4);

        cv::Mat p_inv = p.inv();
        cv::Mat t = p_inv.rowRange(0,3).col(3).clone();
        float s = p_inv.at<float>(3,3);
        t.at<float>(0)/=s;
        t.at<float>(1)/=s;
        t.at<float>(2)/=s;

        return t;
    }


	cv::Mat SkewSymmetry(cv::Mat v)
	{
		if(v.type() == CV_32F){
			return SkewSymmetry_Impl<float>(v);
		}
		else if(v.type() == CV_64F){
			return SkewSymmetry_Impl<double>(v);
		}

		return cv::Mat();
	}

	cv::Mat getR(cv::Mat T)
	{	
		cv::Mat R = T(cv::Rect(0,0,3,3));
		R = R/T.at<float>(3,3);

		return R;
	}
	cv::Mat getT(cv::Mat T)
	{
		cv::Mat t = T(cv::Rect(3,0,1,3));
		t/=T.at<float>(3,3);

		return t;	
	}
	cv::Mat computeF(cv::Mat T1, cv::Mat k1
			, cv::Mat T2, cv::Mat k2)
	{
		cv::Mat dT = T2 * T1.inv();
		cv::Mat R12 = getR(dT);
		cv::Mat t12 = getT(dT);
		cv::Mat t12x = SkewSymmetry(t12);

		cv::Mat F = k1.t().inv() * t12x * R12 * k2.inv();
		return F;
	}

	cv::Mat makeT0()
	{
		cv::Mat T = cv::Mat::eye(4,4,CV_32F);
		return T;
	}

	cv::Mat makeT(cv::Mat R, cv::Mat t)
	{
		cv::Mat T = cv::Mat::eye(4,4,CV_32F);
        R.copyTo(T.rowRange(0,3).colRange(0,3));
        t.copyTo(T.rowRange(0,3).col(3));

		return T;
	}

	cv::Mat makeP0(const cv::Mat k)
	{
		cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
		k.copyTo(P1.rowRange(0,3).colRange(0,3));

	
		return P1;
	}

	cv::Mat makeP(const cv::Mat& k
			, const cv::Mat& R, const cv::Mat& t)
	{
		cv::Mat P2(3,4,CV_32F);
		R.copyTo(P2.rowRange(0,3).colRange(0,3));
		t.copyTo(P2.rowRange(0,3).col(3));
		P2 = k*P2;

		return P2;
	}

	float checkF(cv::Mat F12, cv::Mat _x1, cv::Mat _x2)
	{
		cv::Mat x1 = (cv::Mat_<float>(1,3)<<
				_x1.at<float>(0), _x1.at<float>(1),1); 
		cv::Mat x2 = (cv::Mat_<float>(1,3)<<
				_x2.at<float>(0), _x2.at<float>(1),1);
		cv::Mat v= x1* F12 *x2.t();
		return v.at<float>(0,0);
	}

	float checkF(cv::Mat F12, const cv::Point2f& _x1, const cv::Point2f& _x2)
	{
		cv::Mat x1 = (cv::Mat_<float>(1,3)<<
				_x1.x, _x1.y,1); 
		cv::Mat x2 = (cv::Mat_<float>(1,3)<<
				_x2.x, _x2.y,1);
		cv::Mat v= x1* F12 *x2.t();
		return v.at<float>(0,0);

	}

	int OrbDescDis(const cv::Mat &a, const cv::Mat &b)
	{
		const int *pa = a.ptr<int32_t>();
		const int *pb = b.ptr<int32_t>();

		int dist=0;

		for(int i=0; i<8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}

		return dist;
	}


	cv::Mat makeP(cv::Mat T, cv::Mat k)
	{
		return makeP(k, getR(T), getT(T));
	}
	
	
	std::vector<cv::Point2f> to_vector_point2f(
			const std::vector<cv::KeyPoint>& kpts)
	{
		std::vector<cv::Point2f> pts(kpts.size());
		for(std::size_t i = 0; i< kpts.size(); i++){
			pts[i] = kpts[i].pt;
		}
		return pts;
	}

	
	cv::Point3f to_point3f(cv::Mat p)
	{
		cv::Point3f p3;
		p3.x = p.at<float>(0);
		p3.y = p.at<float>(1);
		p3.z = p.at<float>(2);

		return p3;
	}


	double sampsonDistance(cv::Mat F
			,const cv::Point2f& pt1
			, const cv::Point2f& pt2)
	{
		cv::Mat m1=(cv::Mat_<double>(3,1)<<pt1.x,pt1.y,1);		
		cv::Mat m2 = (cv::Mat_<double>(3,1)<<pt2.x, pt2.y,1);

		return sampsonDistance(F, m1, m2);
	}

	double sampsonDistance(cv::Mat F
			, cv::Mat m1, cv::Mat m2)
	{
		cv::Mat r = m2.t() * F * m1;
		cv::Mat a = F * m1;
		cv::Mat b = F * m2;

		double e = r.at<double>(0) * r.at<double>(0)/
			( a.at<double>(0) * a.at<double>(0) + a.at<double>(1) * a.at<double>(1) 
		   + b.at<double>(0) * b.at<double>(0) + b.at<double>(1) * b.at<double>(1)
	   	   );
		return e;
	
	}

}//algo
