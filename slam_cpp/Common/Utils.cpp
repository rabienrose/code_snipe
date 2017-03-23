#include "Utils.h"

namespace algo{
    cv::Mat from34To44(cv::Mat T)
    {
        if (T.rows <4){
            cv::Mat m44 = cv::Mat::eye(4,4,T.type());
            T.copyTo(m44.rowRange(0, 3));
            return m44;
        }
        return T;
    }
    
	cv::Mat getR(cv::Mat T)
	{
        cv::Mat t44 = from34To44(T);
		cv::Mat R = t44(cv::Rect(0,0,3,3));
		R = R/t44.at<float>(3,3);

		return R;
	}
	cv::Mat getT(cv::Mat T)
	{
        cv::Mat t44 = from34To44(T);
		cv::Mat t = t44(cv::Rect(3,0,1,3));
		t/=t44.at<float>(3,3);

		return t;	
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
    
    cv::Mat p3ftoMat(cv::Point3f p3)
    {
        cv::Mat posiM(3,1, CV_64F);
        posiM.at<double>(0) = p3.x;
        posiM.at<double>(1) = p3.y;
        posiM.at<double>(2) = p3.z;
        
        return posiM;
    }
    
    cv::Mat pf3toMat(cv::Point3f p3)
    {
        cv::Mat posiM(4,1, CV_64FC1);
        posiM.at<double>(0) = p3.x;
        posiM.at<double>(1) = p3.y;
        posiM.at<double>(2) = p3.z;
        posiM.at<double>(3) = 1;
        return posiM;
    }
    
    std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
    {
        std::vector<cv::Mat> vDesc;
        vDesc.reserve(Descriptors.rows);
        for (int j=0;j<Descriptors.rows;j++)
            vDesc.push_back(Descriptors.row(j));
        
        return vDesc;
    }
    
    g2o::SE3Quat toSE3Quat(const cv::Mat & rt)
    {
		cv::Mat cvT = rt.clone();
		if(cvT.type() == CV_64FC1) {
			cvT.convertTo(cvT, CV_32FC1);
		}
		
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
			cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
			cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);
        
        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
        
        return g2o::SE3Quat(R,t);
    }
    
    cv::Mat toCvMat(const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        return toCvMat(eigMat);
    }
    
    cv::Mat toCvMat(const g2o::Sim3 &Sim3)
    {
        Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = Sim3.translation();
        double s = Sim3.scale();
        return toCvSE3(s*eigR,eigt);
    }
    
    cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m)
    {
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=m(i,j);
        
        return cvMat.clone();
    }
    
    cv::Mat toCvMat(const Eigen::Matrix3d &m)
    {
        cv::Mat cvMat(3,3,CV_32F);
        for(int i=0;i<3;i++)
            for(int j=0; j<3; j++)
                cvMat.at<float>(i,j)=m(i,j);
        
        return cvMat.clone();
    }
    
    cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m)
    {
        cv::Mat cvMat(3,1,CV_32F);
        for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);
        
        return cvMat.clone();
    }
    
    cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
    {
        cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                cvMat.at<float>(i,j)=R(i,j);
            }
        }
        for(int i=0;i<3;i++)
        {
            cvMat.at<float>(i,3)=t(i);
        }
        
        return cvMat.clone();
    }
    
    Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector)
    {
        Eigen::Matrix<double,3,1> v;
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
        
        return v;
    }
    
    Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint)
    {
        Eigen::Matrix<double,3,1> v;
        v << cvPoint.x, cvPoint.y, cvPoint.z;
        
        return v;
    }
    
    Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
    {
        Eigen::Matrix<double,3,3> M;
        
        M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
        cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
        cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);
        
        return M;
    }
    
    std::vector<float> toQuaternion(const cv::Mat &M)
    {
        Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
        Eigen::Quaterniond q(eigMat);
        
        std::vector<float> v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();
        
        return v;
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
    
    //get the scale factor set
    const std::vector<float>& calcScaleSet(float scale, int nlevel)
    {
        static std::vector<float> scaleSet = {1.0f};
        static float last_scale = 1.0f;
        static int last_nlevel = 1;
        if(scale == last_scale && nlevel == last_nlevel) {
            return scaleSet;
        }
        
        scaleSet.resize(nlevel);
        
        //calculate
        for (int l = 1; l < nlevel; l++){
            scaleSet[l] = scaleSet[l - 1] * scale;
        }
        
        //update
        last_scale = scale;
        last_nlevel = nlevel;
        
        return scaleSet;
    }
    
    //get the square of scale factor set
    const std::vector<float>& calcScaleSquareSet(float scale, int nlevel)
    {
        static std::vector<float> scaleSquareSet = {1.0f};
        static float last_scale = 1.0f;
        static int last_nlevel = 1;
        if(scale == last_scale && nlevel == last_nlevel) {
            return scaleSquareSet;
        }
        
        scaleSquareSet.resize(nlevel);
        
        //calculate
        std::vector<float> scaleSet = calcScaleSet(scale, nlevel);
        for (int l = 1; l < nlevel; l++){
            scaleSquareSet[l] = scaleSet[l] * scaleSet[l];
        }
        
        //update
        last_scale = scale;
        last_nlevel = nlevel;
        
        return scaleSquareSet;
    }
    
    //get the inverse of scale factor set
    const std::vector<float>& calcScaleSetInv(float scale, int nlevel)
    {
        static std::vector<float> scaleSetInv = {1.0f};
        static float last_scale = 1.0f;
        static int last_nlevel = 1;
        if(scale == last_scale && nlevel == last_nlevel) {
            return scaleSetInv;
        }
        
        scaleSetInv.resize(nlevel);
        
        //calculate
        std::vector<float> scaleSet = calcScaleSet(scale, nlevel);
        for (int l = 1; l < nlevel; l++){
            scaleSetInv[l] = 1.0 / scaleSet[l];
        }
        
        //update
        last_scale = scale;
        last_nlevel = nlevel;
        
        return scaleSetInv;
    }
    
    //get the inverse square of scale factor set
    const std::vector<float>& calcScaleSquareSetInv(float scale, int nlevel)
    {
        static std::vector<float> scaleSquareSetInv = {1.0f};
        static float last_scale = 1.0f;
        static int last_nlevel = 1;
        if(scale == last_scale && nlevel == last_nlevel) {
            return scaleSquareSetInv;
        }
        
        scaleSquareSetInv.resize(nlevel);
        
        //calculate
        std::vector<float> scaleSquareSet = calcScaleSquareSet(scale, nlevel);
        for (int l = 1; l < nlevel; l++){
            scaleSquareSetInv[l] = 1.0 / scaleSquareSet[l];
        }
        
        //update
        last_scale = scale;
        last_nlevel = nlevel;
        
        return scaleSquareSetInv;
    }
    
    cv::Mat getCamCenterFromPose(cv::Mat pose)
    {
        cv::Mat Rcw = pose.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = pose.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        return -Rwc*tcw;
    }
    
    template<typename _T>
    cv::Mat SkewSymmetry_Impl(cv::Mat v)
    {
        cv::Mat m =  (cv::Mat_<_T>(3,3) <<
                      0, -v.at<_T>(2), v.at<_T>(1),
                      v.at<_T>(2), 0, -v.at<_T>(0),
                      -v.at<_T>(1), v.at<_T>(0), 0);
        
        return m;
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

}//algo
