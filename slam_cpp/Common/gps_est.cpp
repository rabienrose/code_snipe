#include "Utils.h"

#include "gps_est.h"

namespace CommonAlgo{
    bool computeTransform(std::vector<cv::Point3f>& vmRelGps, std::vector<cv::Mat>& vCameraPose, cv::Mat& transform, similarityInfo& score)
    {
        std::vector<cv::Mat> vSegCenter,vTempCenter;
        std::vector<cv::Mat> vSegGps,vTempGps;
        std::vector<cv::Mat> vSegCameraPose;
        std::vector<cv::Mat> vSegGpsSim3;
        for(int j=0;j<=vCameraPose.size();j++)
        {
            cv::Mat GPS = (cv::Mat_<float>(3, 1) << vmRelGps[j].x, vmRelGps[j].y, vmRelGps[j].z);
            cv::Mat Rcw = vCameraPose[j].rowRange(0,3).colRange(0,3);
            cv::Mat tcw = vCameraPose[j].rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat C = -Rwc*tcw;
            cv::Mat Pose = vCameraPose[j];
            cv::Mat GPSsim3 = (cv::Mat_<float>(3, 1) << vmRelGps[j].x, vmRelGps[j].z, vmRelGps[j].y);
            vSegGpsSim3.push_back(GPSsim3);
            vSegCenter.push_back(C);
            vSegGps.push_back(GPS);
            vSegCameraPose.push_back(Pose);
        }
        
        // sim3 transform:
        if(computeSim3(vSegCenter, vSegGpsSim3, transform))
        {
            std::cout<<"transform="<<transform<<std::endl;
            Slam2Gps(transform,vSegCameraPose);
            //saveSlamGpsKML(i,vSegGps,vSegCameraPose);
            computeSimilarity(vSegGps,vSegCameraPose,score);
            return true;
        }
        else
        return false;
    }
    
    void computeSimilarity(const std::vector<cv::Mat>& vGps, const std::vector<cv::Mat>& vCameraPose, similarityInfo& si)
    {
        if(vGps.size() != vCameraPose.size())
        {
            printf("gps and pose size is not equal.\n");
            return ;
        }
        
        int size = (int) vGps.size();
        int size2 = 2 * size;
        
        double a0 = 0, a1 = 0, b0 = 0, b1 = 0;
        cv::Mat pad_pos(1, size2, CV_32F);
        cv::Mat pad_gps(1, size2, CV_32F);
        cv::Mat mean_pos(1, size, CV_32F);
        cv::Mat mean_gps(1, size, CV_32F);
        
        mean_pos.setTo(0);
        mean_gps.setTo(0);
        
        for(int i = 0; i < size; ++i)
        {
            cv::Mat pos = -vCameraPose[i].rowRange(0,3).colRange(0,3).t() * vCameraPose[i].rowRange(0,3).col(3);
            const cv::Mat& gps = vGps[i];
            pad_pos.at<float>(0,i*2) = pos.at<float>(0,0), pad_pos.at<float>(0,i*2+1) = pos.at<float>(2,0);
            pad_gps.at<float>(0,i*2) = gps.at<float>(0,0), pad_gps.at<float>(0,i*2+1) = gps.at<float>(1,0);
            
            mean_pos.at<float>(0,i) = (pos.at<float>(0,0) + pos.at<float>(2,0)) / 2.f;
            mean_gps.at<float>(0,i) = (gps.at<float>(0,0) + gps.at<float>(1,0)) / 2.f;
            a0 += pos.at<float>(0,0);
            a1 += pos.at<float>(2,0);
            b0 += gps.at<float>(0,0);
            b1 += gps.at<float>(1,0);
        }
        a0 /= size;
        a1 /= size;
        b0 /= size;
        b1 /= size;
        
        float pos0, pos1, gps0, gps1;
        float sim00 = 0, sim01 = 0, _sim00 = 0, _sim01 = 0, sim00_ = 0, sim01_ = 0;
        float sim10 = 0, sim11 = 0, _sim10 = 0, _sim11 = 0, sim10_ = 0, sim11_ = 0;
        double p0 = 0, p1 = 0, g0 = 0, g1 = 0;
        
        si.sim1 = 0;
        si.sim2 = 0;
        si.sim3 = 0;
        si.sim4 = 0;
        
        for(int i = 0; i < size; ++i)
        {
            cv::Mat pos = -vCameraPose[i].rowRange(0,3).colRange(0,3).t() * vCameraPose[i].rowRange(0,3).col(3);
            const cv::Mat& gps = vGps[i];
            pos0 = pos.at<float>(0,0);
            pos1 = pos.at<float>(2,0);
            gps0 = gps.at<float>(0,0);
            gps1 = gps.at<float>(1,0);
            
            p0 = pos0 - a0;
            p1 = pos1 - a1;
            g0 = gps0 - b0;
            g1 = gps1 - b1;
            
            sim00 += p0 * g0;
            sim01 += p1 * g1;
            sim00_ += (p0*p0);
            _sim00 += (g0*g0);
            sim01_ += (p1*p1);
            _sim01 += (g1*g1);
            
            sim10 += pos0 * gps0;
            sim11 += pos1 * gps1;
            sim10_ += (pos0*pos0);
            _sim10 += (gps0*gps0);
            sim11_ += (pos1*pos1);
            _sim11 += (gps1*gps1);
            
        }
        
        si.sim1 = (fabs(sim00) / sqrt(sim00_*_sim00) + fabs(sim01) / sqrt(sim01_*_sim01)) / 2.f;
        si.sim2 = (fabs(sim10) / sqrt(sim10_*_sim10) + fabs(sim11) / sqrt(sim11_*_sim11)) / 2.f;
        
        sim01 = 0, sim11 = 0;
        
        for(int i = 0; i < size; ++i)
        {
            sim00_ = 0, sim00 = 0, sim10_ = 0; sim10 = 0;
            
            cv::Mat pos = -vCameraPose[i].rowRange(0,3).colRange(0,3).t() * vCameraPose[i].rowRange(0,3).col(3);
            const cv::Mat& gps = vGps[i];
            pos0 = pos.at<float>(0,0);
            pos1 = pos.at<float>(2,0);
            gps0 = gps.at<float>(0,0);
            gps1 = gps.at<float>(1,0);
            
            p0 = pos0 - mean_pos.at<float>(0,i);
            p1 = pos1 - mean_pos.at<float>(0,i);
            g0 = gps0 - mean_gps.at<float>(0,i);
            g1 = gps1 - mean_gps.at<float>(0,i);
            
            sim00_ = fabs((p0 * g0) + (p1 * g1));
            sim00 = sqrt((p0*p0) + (p1*p1)) * sqrt((g0*g0) + (g1*g1));
            
            sim01 += sim00_ / sim00;
            
            sim10_ = pos0 * gps0 + pos1 * gps1;
            sim10 = sqrt((pos0*pos0) + (pos1*pos1)) * sqrt((gps0*gps0) + (gps1*gps1));
            
            sim11 += sim10_ / sim10;
            
        }
        
        si.sim3 = sim01 / size;
        si.sim4 = sim11 / size;
        
        si.sim5 = 0;
        si.sim6 = 0;
        
        a0 = 0, a1 = 0;
        for(int i = 0; i < size2; ++i)
        {
            a0 += pad_pos.at<float>(0,i);
            a1 += pad_gps.at<float>(0,i);
        }
        a0 /= size2;
        a1 /= size2;
        sim00 = 0, sim00_ = 0, _sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i)-a0;
            b1 = pad_gps.at<float>(0,i)-a1;
            sim00 += b0 * b1;
            sim00_ += b0 * b0;
            _sim00 += b1 * b1;
        }
        si.sim5 = fabs(sim00) / sqrt(sim00_ * _sim00);
        
        
        sim00 = 0, sim00_ = 0, _sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i);
            b1 = pad_gps.at<float>(0,i);
            sim00 += b0 * b1;
            sim00_ += b0 * b0;
            _sim00 += b1 * b1;
        }
        si.sim6 = fabs(sim00) / sqrt(sim00_ * _sim00);
        
        si.sim7 = 0;
        si.sim8 = 0;
        si.sim9 = 0;
        
        sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i);
            b1 = pad_gps.at<float>(0,i);
            a0 = fabs(b0-b1);
            sim00 += a0 / (1.f + expf(-a0));
        }
        si.sim7 = sim00;
        
        sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i);
            b1 = pad_gps.at<float>(0,i);
            sim00 += fabs((b0-b1) / (b0+b1));
        }
        si.sim8 = sim00;
        
        sim00_ = 0, _sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i);
            b1 = pad_gps.at<float>(0,i);
            sim00_ += std::min(b0, b1);
            _sim00 += std::max(b0, b1);
        }
        si.sim9 = sim00_ / _sim00;
        
        si.sim0 = 0;
        sim00 = 0;
        for(int i = 0; i < size2; ++i)
        {
            b0 = pad_pos.at<float>(0,i);
            b1 = pad_gps.at<float>(0,i);
            sim00 += fabs(b0-b1) / b1;
        }
        
        si.sim0 = 1.f - sim00 / size2;
        printf("similarity is: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f.\n", si.sim0, si.sim1, si.sim2, si.sim3, si.sim4, si.sim5, si.sim6, si.sim7, si.sim8, si.sim9);
    }

    
    void saveSlamGpsKML(int counter,std::vector<cv::Mat>& vGps,std::vector<cv::Mat>& vCameraPose)
    {
    }
    
    bool computeSim3(std::vector<cv::Mat>& vCameraCenter, std::vector<cv::Mat>& vGps,cv::Mat &Transform)
    {
        int N = vCameraCenter.size();
        if(vCameraCenter.size()!=vGps.size())
        {
            std::cout<<"error :the gps.size() != camerapose.size()"<<std::endl;
            return false;
        }
        
        cv::Mat mGps = cv::Mat::zeros(3,N,CV_32F);
        cv::Mat mCenter = cv::Mat::zeros(3,N,CV_32F);
        
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
            
            N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
            N12 = M.at<float>(1,2)-M.at<float>(2,1);
            N13 = M.at<float>(2,0)-M.at<float>(0,2);
            N14 = M.at<float>(0,1)-M.at<float>(1,0);
            N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
            N23 = M.at<float>(0,1)+M.at<float>(1,0);
            N24 = M.at<float>(2,0)+M.at<float>(0,2);
            N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
            N34 = M.at<float>(1,2)+M.at<float>(2,1);
            N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);
            
            N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                 N12, N22, N23, N24,
                 N13, N23, N33, N34,
                 N14, N24, N34, N44);
            
            
            // Step 4: Eigenvector of the highest eigenvalue
            
            cv::Mat eval, evec;
            
            cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation
            
            cv::Mat vec(1,3,evec.type());
            (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)
            
            // Rotation angle. sin is the norm of the imaginary part, cos is the real part
            double ang=atan2(norm(vec),evec.at<float>(0,0));
            
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
                    den+=aux_P3.at<float>(i,j);
                }
            }
            
            float ms12i = nom/den;
            
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
            
            Transform = cv::Mat::eye(4,4,CV_32F);
            sR.copyTo(Transform.rowRange(0,3).colRange(0,3));
            mt12i.copyTo(Transform.rowRange(0,3).col(3));
        }
        
        if(Transform.empty())
        return false;
        else
        return true;
    }
    
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,CV_REDUCE_SUM);
        C = C/P.cols;
        
        for(int i=0; i<P.cols; i++)
        {
            Pr.col(i)=P.col(i)-C;
        }
    }
    
    
    void Slam2Gps(cv::Mat &Transform, std::vector<cv::Mat> &vCameraPose)
    {
        for (int i = 0; i < vCameraPose.size(); ++i)
        {
            cv::Mat P = vCameraPose[i].clone();
            
            if(P.rows ==3)
            {
                cv::Mat temp = cv::Mat::eye(4,4,CV_32F);
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
            
            cv::Mat newPose = cv::Mat::eye(4,4,CV_32F);
            
            newR.copyTo(newPose.rowRange(0,3).colRange(0,3));
            newT.copyTo(newPose.rowRange(0,3).col(3));
            
            newPose.copyTo(vCameraPose[i]);
            
        }
    }
    
    bool CorrectRotation(cv::Mat &R, double &scale, cv::Mat &normR)
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
