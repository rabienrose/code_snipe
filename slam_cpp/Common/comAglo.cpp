#include "Utils.h"

#include "comAglo.h"
#include "utilityFuns.h"

using namespace roadDBCore;

namespace CommonAlgo{
	
    std::unique_ptr<transformTools> transformTools::m_pInstance = nullptr;
	
    EpipolarFilter::EpipolarFilter()
    {
        
    }
    
    void EpipolarFilter::setK(cv::Mat k)
    {
        if(k.type() == CV_32F){
            k.convertTo(k, CV_64F);
        }
        cv::Mat c=(cv::Mat_<double>(3,1)<<0,0,1);
        cv::Mat e = k * c;
        epipole_.x = e.at<double>(0);
        epipole_.y = e.at<double>(1);
        
        //LOG_INFO(glog)<<"epipole = "<<epipole_;
    }
    
    bool EpipolarFilter::setPts(
                                const std::vector<cv::Point2f>& pts1
                                , const std::vector<cv::Point2f>& pts2)
    {
        pts1_.clear();
        pts2_.clear();
        if(pts1.size() != pts2.size()){
            return false;
        }
        
        pts1_ = pts1;
        pts2_ = pts2;
        return true;
    }
    
    bool EpipolarFilter::operator()()
    {
        inliers_.resize(pts1_.size(), false);
        
        for(std::size_t i =0 ;i< inliers_.size(); i++)
        {
            if(_checkInlier(pts1_[i], pts2_[i], epipole_))
                inliers_[i] = true;
        }
        
        return true;
    }
    
    bool EpipolarFilter::_checkInlier(const cv::Point2f& p1
                                      , const cv::Point2f& p2
                                      , const cv::Point2f& e)
    {
        
        cv::Mat a = (cv::Mat_<double>(2,1)<<p1.x-e.x,p1.y-e.y);
        cv::Mat b = (cv::Mat_<double>(2,1)<<p2.x-e.x,p2.y-e.y);
        
        double cos_theta = a.dot(b) / ( cv::norm(a) * cv::norm(b) );
        if(cos_theta < 0){
            return false;
        }
        
        return true;	
    }

	transformTools::transformTools()
	{
	    pIGPS_ = algo::AlgoGPS::createInstance();
	}
	
	
	
	void transformTools::convertToAbsCamCenter(std::vector<cv::Mat> &vecCamCenters)
	{
		std::vector<cv::Mat>::iterator itCC = vecCamCenters.begin();
		for ( ; itCC != vecCamCenters.end(); ++itCC )
		{
			cv::Mat &camCenter = *itCC;
			if ( !camCenter.empty() )
			{
				cv::Point3d relPos, // relative position
							absPos; // absolute position

				relPos.x = camCenter.at<double>(0, 0);
				relPos.y = camCenter.at<double>(2, 0);
				relPos.z = camCenter.at<double>(1, 0);

				cv::Point3d refgps;
				pIGPS_->getRefPos(refgps);
				
				roadDBCore::calcGpsFromRelLocation(refgps, relPos, absPos);

				camCenter.at<double>(0, 0) = absPos.x;
				camCenter.at<double>(2, 0) = absPos.y;
				camCenter.at<double>(1, 0) = absPos.z;
			}
		}
	}
	CommonAlgo::transformTools* transformTools::createInstance()
	{
        if(!m_pInstance)
            m_pInstance = std::unique_ptr<transformTools>(new transformTools());
        return m_pInstance.get();
	}
	
    void computeEpipolarline(const cv::Point2f &pt, const cv::Mat &F12, float &a, float &b, float &c)
    {
        // Epipolar line in second image l = x1'F12 = [a b c]
        float x = pt.x, y = pt.y;
        
        a = x*F12.at<float>(0,0) + y*F12.at<float>(1,0)+F12.at<float>(2,0);
        b = x*F12.at<float>(0,1) + y*F12.at<float>(1,1)+F12.at<float>(2,1);
        c = x*F12.at<float>(0,2) + y*F12.at<float>(1,2)+F12.at<float>(2,2);
    }
    
    bool checkDistEpipolarLine(const cv::Point2f &pt, float sigma, const float &a, const float &b, const float &c, const float &denInverse)
    {
        //const float num = a*kp2.pt.x+b*kp2.pt.y+c;
        float num = a;
        num *= pt.x;
        num += b*pt.y;
        num += c;
        
        const float dsqr = num*num*denInverse;
        
        return dsqr < 3.84*sigma;
    }
    
    void computeThreeMaxima(std::vector<std::vector<int>>& histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;
        
        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }
        
        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }
    
    double getlevelSigma(double octave, double sf)
    {
        return std::pow(sf, octave);
    }
    
    double predictScaleLevel(double d, double l0_d, double sf)
    {
        if(sf<=1){
            return 0;
        }
        double scale = 1.0/sf;
        return log(l0_d/d)/log(scale);
    }
    
    bool checkScaleLevel(double d1, int octave1, double sf1
                         , double d2, int octave2, double sf2
                         , double thr)
    {
        if(sf1 <=1 || sf2 <=1){
            return false;
        }
        if(d1 <=0 || d2<=0){
            return false;
        }
        
        //LOG_INFO(glog)<<"****";
        //LOG_INFO(glog)<<"sf1 = "<<sf1;
        //LOG_INFO(glog)<<"d1 = "<<d1;
        
        double l0_d1 = d1 * std::pow(1.0/sf1, octave1);
        double l0_d2 = d2 * std::pow(1.0/sf2, octave2);
        
        //LOG_INFO(glog)<<"l0_d1 = "<<l0_d1;
        //LOG_INFO(glog)<<"octave1 = "<<octave1;
        
        double pre_o1 = predictScaleLevel(d1, l0_d2, sf1);
        double pre_o2 = predictScaleLevel(d2, l0_d1, sf2);
        
        double bundle = thr;
        if ( std::abs(octave1 - pre_o1) <= bundle
            && std::abs(octave2 - pre_o2) <= bundle )
            return true;
        
        return false;
    }

	bool assurePathExists(const std::string &strPath)
	{
		bool bRet = false;

		bf::path targetPath = bf::path(strPath);
		if ( !targetPath.empty() )
		{
			if ( bf::exists(targetPath) )
			{
				bRet = true;
			}
			else
			{
				bRet = bf::create_directories(targetPath);
			}
		}
		else
		{
			// empty string represents current directory
			bRet = true;
		}

		return bRet;
	}
                                                             
	void transformTools::convertToAbsGps(cv::Point3d refGPSP3d, std::vector<algo::GPSData> &vecGPS)
	{
		std::vector<algo::GPSData>::iterator itGPS = vecGPS.begin();
		for ( ; itGPS != vecGPS.end(); ++itGPS )
		{
			convertToAbsGps(refGPSP3d, *itGPS);
		}
	}
	void transformTools::convertToAbsGps(cv::Point3d refGPSP3d, algo::GPSData &gpsData)
	{
		cv::Point3d relPos, // relative position
			 absPos; // absolute position

		relPos.x = gpsData.lon;
		relPos.y = gpsData.lat;
		relPos.z = gpsData.alt;

		// relative -> absolute
		calcGpsFromRelLocation(refGPSP3d, relPos, absPos);

		// position back to GPSData
		gpsData.lon = absPos.x;
		gpsData.lat = absPos.y;
		gpsData.alt = absPos.z;
	}
	
    
    bool transformTools::computeSim3(std::vector<cv::Mat>& vCameraCenter, std::vector<cv::Mat>& vGps,cv::Mat &Transform)
    {
        int N = vCameraCenter.size();
        if(vCameraCenter.size()!=vGps.size())
        {
            std::cout<<"error :the gps.size() != camerapose.size()"<<std::endl;
            return false;
        }
        
        cv::Mat mGps = cv::Mat::zeros(3,N,CV_64FC1);
        cv::Mat mCenter = cv::Mat::zeros(3,N,CV_64FC1);
        
        for(int i=0;i<N;i++)
        {
            cv::Mat gps = vGps[i].clone();
            gps.copyTo(mGps.col(i));
            cv::Mat center = vCameraCenter[i].clone();
            center.copyTo(mCenter.col(i));
        }
        
        {
            // Custom implementation of:
            // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions
            
            // Step 1: Centroid and relative coordinates
            
            cv::Mat Pr1(mGps.size(),mGps.type()); // Relative coordinates to centroid (set 1)
            cv::Mat Pr2(mCenter.size(),mCenter.type()); // Relative coordinates to centroid (set 2)
            cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
            cv::Mat O2(3,1,Pr2.type()); // Centroid of P2
            
            ComputeCentroid(mGps,Pr1,O1);
            ComputeCentroid(mCenter,Pr2,O2);
            
            // Step 2: Compute M matrix
            
            cv::Mat M = Pr2*Pr1.t();
            
            // Step 3: Compute N matrix
            
            double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;
            
            cv::Mat N(4,4,mGps.type());
            
            N11 = M.at<double>(0,0)+M.at<double>(1,1)+M.at<double>(2,2);
            N12 = M.at<double>(1,2)-M.at<double>(2,1);
            N13 = M.at<double>(2,0)-M.at<double>(0,2);
            N14 = M.at<double>(0,1)-M.at<double>(1,0);
            N22 = M.at<double>(0,0)-M.at<double>(1,1)-M.at<double>(2,2);
            N23 = M.at<double>(0,1)+M.at<double>(1,0);
            N24 = M.at<double>(2,0)+M.at<double>(0,2);
            N33 = -M.at<double>(0,0)+M.at<double>(1,1)-M.at<double>(2,2);
            N34 = M.at<double>(1,2)+M.at<double>(2,1);
            N44 = -M.at<double>(0,0)-M.at<double>(1,1)+M.at<double>(2,2);
            
            N = (cv::Mat_<double>(4,4) << N11, N12, N13, N14,
                 N12, N22, N23, N24,
                 N13, N23, N33, N34,
                 N14, N24, N34, N44);
            
            
            // Step 4: Eigenvector of the highest eigenvalue
            
            cv::Mat eval, evec;
            
            cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation
            
            cv::Mat vec(1,3,evec.type());
            (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)
            
            // Rotation angle. sin is the norm of the imaginary part, cos is the real part
            double ang=atan2(norm(vec),evec.at<double>(0,0));
            
            vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half
            
            cv::Mat mR12i;
            mR12i.create(3,3,mGps.type());
            
            cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis
            
            // Step 5: Rotate set 2
            
            cv::Mat P3 = mR12i*Pr2;
            
            // Step 6: Scale
            
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;
            
            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<double>(i,j);
                }
            }
            
            double ms12i = nom/den;
            
            // Step 7: Translation
            cv::Mat mt12i;
            mt12i.create(1,3,mGps.type());
            mt12i = O1 - ms12i*mR12i*O2;
            
            // Step 8: Transformation
            
            // Step 8.1 T12
            cv::Mat mT12i = cv::Mat::eye(4,4,mGps.type());
            
            cv::Mat sR = ms12i*mR12i;
            
            sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
            mt12i.copyTo(mT12i.rowRange(0,3).col(3));
            
            // Step 8.2 T21
            
            cv::Mat mT21i = cv::Mat::eye(4,4,mGps.type());
            
            cv::Mat sRinv = (1.0/ms12i)*mR12i.t();
            
            Transform = cv::Mat::eye(4,4,CV_64FC1);
            sR.copyTo(Transform.rowRange(0,3).colRange(0,3));
            mt12i.copyTo(Transform.rowRange(0,3).col(3));
        }
        
        if(Transform.empty())
		{
			std::cout<<"computeSim3 Transform empty!"<<std::endl;
            return false;
		}
        else
            return true;
    }
	
    
    void transformTools::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,CV_REDUCE_SUM);
        C = C/P.cols;
        
        for(int i=0; i<P.cols; i++)
        {
            Pr.col(i)=P.col(i)-C;
        }
    }

    void transformTools::Slam2Gps(cv::Mat &Transform, std::vector<cv::Mat> &vCameraPose)
    {
        for (int i = 0; i < vCameraPose.size(); ++i)
        {
            cv::Mat P = vCameraPose[i].clone();
            
            if(P.rows ==3)
            {
                cv::Mat temp = cv::Mat::eye(4,4,CV_64FC1);
                P.copyTo(temp.rowRange(0, 3).colRange(0, 4));
                P = temp.clone();
            }
            
            cv::Mat poseInv = P.inv();
            cv::Mat newPoseInv = Transform * poseInv;

            cv::Mat pose = newPoseInv.inv();
            
            cv::Mat R = pose.rowRange(0,3).colRange(0,3);
            cv::Mat T = pose.rowRange(0,3).col(3);
            
            cv::Mat newR;
            double scale = 0.0;
            CorrectRotation(R, scale, newR);
            
            cv::Mat newT = T*scale;
            
            cv::Mat newPose = cv::Mat::eye(4,4,CV_64FC1);
            
            newR.copyTo(newPose.rowRange(0,3).colRange(0,3));
            newT.copyTo(newPose.rowRange(0,3).col(3));
            
            newPose.copyTo(vCameraPose[i]);
            
        }
    }
	
    bool transformTools::CorrectRotation(cv::Mat &R, double &scale, cv::Mat &normR)
    {
        double det = cv::determinant(R);
        double newDet;
        if(det<0)
        {
            newDet = -pow(-det, 1/3.0);
        }
        else
        {
            newDet = pow(det, 1/3.0);
        }
        scale = 1.0/newDet;
        if(std::isnan(scale))
        {
            std::cout<<"R="<<R<<std::endl;
            std::cout<<"nan det="<<det<<std::endl;
            std::cout<<"newDet"<<newDet<<std::endl;
            std::cout<<"error: scale = nan"<<std::endl;
            normR = scale * R;
            return 0;
        }
        
        normR = scale * R;
        return 1;
    }
                                                             
}
