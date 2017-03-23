#include <exception>
#include <typeinfo>
#include <stdexcept>
#include "Utils.h"
#include "NewMPCreator.h"
#include "comAglo.h"
#include "FeatureTool.hpp"
#include <iostream>
#include <fstream>

namespace algo{
    
    NewCloudPointCreator::NewCloudPointCreator(cv::Mat& _obPose, cv::Mat _K, float _scaleFactor, int _descType,const std::vector<float>& _scaleFactors,
                                               std::vector<cv::KeyPoint>& _obKps, std::vector<cv::Mat>& _obDescs, std::vector<bool>& _obMpMask)
    {
        K = _K;
        K_inv = K.inv();
        obR_ = getR(_obPose);
        obT_ = getT(_obPose);
        
        obPose = from34To44(_obPose);
        
        cx = K.at<float>(2,0);
        cy = K.at<float>(2,1);
        fx = K.at<float>(0,0);
        fy = K.at<float>(0,0);
        invfx = 1/fx;
        invfy = 1/fy;
        scaleFactor = _scaleFactor;
        scaleFactors = _scaleFactors;
        descType = _descType;
        
        obKps = _obKps;
        obDescs =_obDescs;
        obMpMask = _obMpMask;
    }
    
    void NewCloudPointCreator::set(cv::Mat _refPose, std::vector<cv::KeyPoint>& _refKps, std::vector<cv::Mat>& _refDescs, std::vector<bool>& _refMpMask)
    {
        rR_ = getR(_refPose);
        rT_ = getT(_refPose);
        rPose = from34To44(_refPose);
        refKps = _refKps;
        refDescs =_refDescs;
        refMpMask = _refMpMask;
    }
    
    bool NewCloudPointCreator::operator()(std::vector<std::pair<size_t, size_t> >& matchedPairs, std::vector<cv::Point3f>& posiList)
    {

        auto mps = _getMatchPair();
        
        typedef struct {
            cv::Mat x3D;
            cv::KeyPoint kp1;
            cv::KeyPoint kp2;
            std::size_t idx1;
            std::size_t idx2;
        } MapInfo;
        
        std::vector<MapInfo> mpInfos;
        
        //std::size_t n =0;
        std::ofstream fout1("/Volumes/chamo/working/matlab_slam/v2.0/unit_test/platform/NewMPCreator/cases/comp1.txt");
        for(std::size_t i = 0; i< mps.size(); i++)
        {
            
            const std::size_t idx_rkf = mps[i].second;
            const std::size_t idx_obkf = mps[i].first;
            
            const cv::KeyPoint& kp1 = refKps[idx_rkf];
            const cv::KeyPoint& kp2 = obKps[idx_obkf];
            fout1<<std::endl;
            fout1<<idx_obkf<<" "<<idx_rkf <<" "<<kp2.pt<<" "<< kp1.pt;
            cv::Mat x1 = (cv::Mat_<float>(3,1)<<
                          (kp1.pt.x - cx) * invfx
                          , (kp1.pt.y - cy)*invfy
                          , 1);
            cv::Mat x2 = (cv::Mat_<float>(3,1)<<
                          (kp2.pt.x-cx) * invfx
                          , (kp2.pt.y-cy)*invfy
                          , 1);
            
            
            if(_checkIsFarPoint(x2, obR_, x1, rR_))
                continue;
            
            cv::Mat x3D = _createx3D2(idx_obkf, idx_rkf);
            
            fout1<<" "<<x3D.t();
            if(x3D.empty())
                continue;
            if(!_checkIsX3DValid(x3D, rPose , K, kp1.pt, kp1.octave, scaleFactor)
               || !_checkIsX3DValid(x3D, obPose, K, kp2.pt, kp2.octave, scaleFactor))
                continue;
            
            MapInfo mpInfo;
            mpInfo.x3D = x3D;
            mpInfo.kp1 = kp1;
            mpInfo.kp2 = kp2;
            mpInfo.idx1 = idx_rkf;
            mpInfo.idx2 = idx_obkf;
            
            mpInfos.push_back(mpInfo);
        }
        fout1.close();
        std::vector<MapInfo> valid_mpInfo;
        
        auto file_x3d = [&](const std::vector<MapInfo>& mpInfos
                            ,std::vector<MapInfo>& v_mpInfo, float threshold)
        {
            if(v_mpInfo.size() >=15)
                return;
            
            v_mpInfo.clear();
            for(std::size_t i =0; i< mpInfos.size(); i++)
            {
                if(!_checkIsValidOctave(mpInfos[i].x3D
                                        , mpInfos[i].kp1.octave
                                        , mpInfos[i].kp2.octave
                                        , threshold))
                    continue;
                v_mpInfo.push_back(mpInfos[i]);
            }
        };
        
        
        file_x3d(mpInfos, valid_mpInfo, 1.9);
        file_x3d(mpInfos, valid_mpInfo, 2.9);
        file_x3d(mpInfos, valid_mpInfo, 3.9);
        file_x3d(mpInfos, valid_mpInfo, 0);
        matchedPairs.resize(valid_mpInfo.size());
        posiList.resize(valid_mpInfo.size());
        for(int i=0; i<valid_mpInfo.size(); i++){
            matchedPairs[i].first = valid_mpInfo[i].idx1;
            matchedPairs[i].second = valid_mpInfo[i].idx1;
            posiList[i].x = valid_mpInfo[i].x3D.at<float>(0);
            posiList[i].y = valid_mpInfo[i].x3D.at<float>(1);
            posiList[i].z = valid_mpInfo[i].x3D.at<float>(2);
        }
        
        return true;
    }
    
    bool NewCloudPointCreator::_checkIsValidOctave(const cv::Mat& x3D
                                                   , int octave_r
                                                   , int octave_ob
                                                   , float thr)
    {
        if(thr<=0)
            return true;
        
        cv::Mat n1 = x3D - getCamCenterFromPose(rPose);
        cv::Mat n2 = x3D - getCamCenterFromPose(obPose);
        
        float d1 = cv::norm(n1);
        float d2 = cv::norm(n2);
        
        if(!CommonAlgo::checkScaleLevel(d1, octave_r, scaleFactor
                            , d2, octave_ob, scaleFactor
                            , thr))
            return false;
        
        return true;
    }
    
    bool NewCloudPointCreator::_checkIsX3DValid(const cv::Mat& x3D
                                                , const cv::Mat& T, const cv::Mat& k
                                                , const cv::Point2f& pt
                                                , int octave, float sf)
    {
        cv::Mat _x3D = (cv::Mat_<float>(4,1)<< x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2),1);
        
        cv::Mat px = T * _x3D;
        px = px.rowRange(0,3);
        if(px.at<float>(2) <= 0){
            return false;
        }
        
        px = k * px;
        
        float _x = pt.x;
        float _y = pt.y;
        float _x2 = px.at<float>(0)/px.at<float>(2);
        float _y2 = px.at<float>(1)/px.at<float>(2);
        
        float e2 = (_x - _x2) * (_x - _x2) + (_y - _y2) * (_y-_y2);
        float sigma = CommonAlgo::getlevelSigma(octave, sf);
        
        if(e2 > 5.991 * sigma * sigma){
            return false;
        }
        
        return true;
    }
    
    int NewCloudPointCreator::SearchForTriangulation(std::vector<std::pair<size_t,size_t> >& vMatchedPairs)
    {
        //Compute epipole in second image
        cv::Mat k32 = K;

        const cv::Mat& Cw = algo::getCamCenterFromPose(obPose);
        cv::Mat R2w = algo::getR(rPose);
        cv::Mat t2w = algo::getT(rPose);
        cv::Mat C2 = R2w*Cw+t2w;
        const float invz = 1.0f/C2.at<float>(2);
        const float ex =fx*C2.at<float>(0)*invz+cx;
        const float ey =fy*C2.at<float>(1)*invz+cy;
        
        pDistFunc calcDistfunc;
        int TH_LOW;
        switch (descType) {
            case 0:
                calcDistfunc = calcDistanceHamming;
                TH_LOW = 256;
                break;
            case 1:
                calcDistfunc = calcDistanceEuler;
                TH_LOW = 4;
                break;
            default:
                calcDistfunc = calcDistanceEuler;
                TH_LOW = 4;
                break;
        }
        
        
        int nmatches=0;
        std::vector<bool> vbMatched2(refKps.size() ,false);
        std::vector<int> vMatches12(obKps.size(),-1);
        
        std::vector<std::vector<int>> rotHist;
        rotHist.resize(HISTO_LENGTH);
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        
        const float factor = 1.0f/HISTO_LENGTH;
        
        
        for (std::size_t i = 0; i < obMpMask.size(); i++)
        {
            const size_t idx1 = i;
            
            if ( (-1 != vMatches12[idx1]) || obMpMask[idx1])
                continue;
            
            const cv::KeyPoint &kp1 = obKps[idx1];
            
            const cv::Mat &d1 = obDescs[idx1];
            
            float bestDist = TH_LOW;
            int bestIdx2 = -1;
            
            float a, b, c, den;

            CommonAlgo::computeEpipolarline(kp1.pt, F12, a, b, c);
            
            den = a*a + b*b;
            if(0.0 == den)
            {
                continue;
            }
            float denseInverse = 1.0/den;
            
            for (std::size_t j = 0; j < refMpMask.size(); j++)
            {
                std::size_t idx2 = j;
                
                const cv::KeyPoint &kp2 = refKps[idx2];
                
                
                if (!CommonAlgo::checkDistEpipolarLine(kp2.pt, scaleFactors[kp2.octave], a,b,c,denseInverse))
                {
                    continue;
                }
                
                // If we have already matched or there is a MapPoint skip
                if (vbMatched2[idx2] || refMpMask[idx2])
                    continue;
                
                const cv::Mat &d2 = refDescs[idx2];
                if(d1.empty()||d2.empty())
                    continue;
                const float dist = calcDistfunc(d1, d2);
                
                if (dist > TH_LOW || dist > bestDist)
                    continue;
                
                const float distex = ex - kp2.pt.x;
                const float distey = ey - kp2.pt.y;

                if (distex * distex + distey * distey < 100 * scaleFactors[kp2.octave])
                {
                    continue;
                }
                
                {
                    bestIdx2 = idx2;
                    bestDist = dist;
                }
            }
            
            if (bestIdx2 >= 0)
            {
                const cv::KeyPoint &kp2 = refKps[bestIdx2];
                vMatches12[idx1] = bestIdx2;
                vbMatched2[bestIdx2] = true;
                nmatches++;
                
                if (mbCheckOrientation)
                {
                    float rot = kp1.angle - kp2.angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = round(rot * factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back(idx1);
                }
            }
        }
        
        
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;
            
            CommonAlgo::computeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
            
            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vMatches12[rotHist[i][j]]=-1;
                    nmatches--;
                }
            }
            
        }
        
        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);
        
        std::ofstream fout("/Volumes/chamo/working/matlab_slam/v2.0/unit_test/platform/NewMPCreator/cases/matches1.txt");
        if(!fout.is_open()) {
            std::cout << "cannot open output stream\n";
            return -1;
        }
        
        fout<<vMatches12.size()<<std::endl;
        for (int i=0; i<vMatches12.size(); i++){
            fout << i << " " << vMatches12[i] << std::endl;
        }
        fout<<std::endl;
        
        fout.close();
        
        for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
        {
            if(vMatches12[i]<0)
                continue;
            vMatchedPairs.push_back(std::make_pair(i,vMatches12[i]));
        }
        
        return nmatches;
    }
    
    bool NewCloudPointCreator::_checkIsFarPoint(const cv::Mat& m1, const cv::Mat& R1
                                                , const cv::Mat& m2, const cv::Mat& R2)
    {
        cv::Mat r1 = R1.t() * m1;
        cv::Mat r2 = R2.t() * m2;
        
        float cos_theta = r1.dot(r2)/ (cv::norm(r1) * cv::norm(r2));
        
        if(cos_theta <= 0 || cos_theta >=0.9998)
            return true;
        
        return false;
    }
    
    std::vector<std::pair<std::size_t, std::size_t> >
    NewCloudPointCreator::_getMatchPair()
    {
        float fNNRatio = 0.6;
        
        F12 = _computeF12(obPose, rPose);
        
        std::vector<std::pair<size_t,size_t> > vMatchedIndices;
        try
        {
            SearchForTriangulation(vMatchedIndices);
        }
        catch(...)
        {
        }
        
        vMatchedIndices = __filter(vMatchedIndices);
        
        return vMatchedIndices;
    }
    
    std::vector<std::pair<std::size_t, std::size_t> >
    NewCloudPointCreator::__filter(std::vector<std::pair<std::size_t, std::size_t> >& mIdx)
    {
        std::vector<cv::Point2f> pts1(mIdx.size()), pts2(mIdx.size());
        for(std::size_t i = 0; i< mIdx.size(); i++)
        {
            pts1[i] = refKps[mIdx[i].first].pt;
            pts2[i] = obKps[mIdx[i].second].pt;
        }
        
        CommonAlgo::EpipolarFilter e_f;
        e_f.setK(K);
        e_f.setPts(pts1, pts2);
        e_f();
        std::vector<bool>& inliers2 = e_f.getInliers();
        
        
        std::vector<std::pair<std::size_t, std::size_t> > f_idx;
        for(std::size_t i = 0; i< inliers2.size(); i++)
        {
            if(inliers2[i])
                f_idx.push_back(mIdx[i]);
        }
        return f_idx;
    }
    
    cv::Mat NewCloudPointCreator::_computeF12(cv::Mat pose1, cv::Mat pose2)
    {
        cv::Mat R1w = getR(pose1);
        cv::Mat t1w = getT(pose1);
        cv::Mat R2w = getR(pose2);
        cv::Mat t2w = getT(pose2);
        
        
        cv::Mat R12 = R1w*R2w.t();
        cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
        
        //cv::Mat t12x = SkewSymmetricMatrix(t12);
        cv::Mat t12x = SkewSymmetry(t12);

        return K.t().inv()*t12x*R12*K.inv();
    }
    
    
    cv::Mat NewCloudPointCreator::_createx3D2(std::size_t idx1, std::size_t idx2)
    {
        cv::Mat Rcw1 = obR_;
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = obT_;
        cv::Mat Tcw1(3,4,CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0,3));
        tcw1.copyTo(Tcw1.col(3));
        
        cv::Mat Rcw2 = rR_;
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = rT_;
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));
        
        const cv::KeyPoint &kp1 = obKps[idx1];
        
        const cv::KeyPoint &kp2 = refKps[idx2];
        
        // Check parallax between rays
        cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx)*invfx, (kp1.pt.y-cy)*invfy, 1.0);
        cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx)*invfx, (kp2.pt.y-cy)*invfy, 1.0);
        
        
        
        cv::Mat x3D;
        // Linear Triangulation Method
        cv::Mat A(4,4,CV_32F);
        A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
        
        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        
        x3D = vt.row(3).t();
        
        
        if(x3D.at<float>(3)==0)
            return cv::Mat();
        
        // Euclidean coordinates
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
        
        cv::Mat x3Dt = x3D.t();
        
        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
        if(z1<=0)
            return cv::Mat();
        
        float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
        if(z2<=0)
            return cv::Mat();
        
        return x3D;
    }
}

