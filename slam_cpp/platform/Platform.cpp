#include <stdio.h>
#include <string.h> /* For strcmp() */
#include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */
#include <vector> /* For STL */
#include "Platform.hpp"
#include "Frame.hpp"
#include "FeatureExtractor.hpp"
#include "GlobalMap.hpp"
#include "ViewInterfaceImp.h"
#include "ORBextractor.h"
#include "paramParser.hpp"
#include <Utils.h>
#include <pangolin/pangolin.h>
#include "utilityFuns.h"
#include <iomanip>
#include "comAglo.h"
#include "algoInterface/IGps.h"

using namespace CommonAlgo;

namespace ygomi {
    
    std::unique_ptr<Platform> Platform::m_pInstance = nullptr;
    bool Platform::m_bInit = false;
    
    Platform::Platform() :
        m_referKFID(-1),
        m_pGlobalMap(nullptr),
        m_pFeatureExtractor(nullptr),
        m_pParamParser(nullptr)
    {
        kf_center_dis2_ =  new algo::Mean;
        std::cout << "ctor is called\n";
        m_newMapPointId.clear();

		std::string debugFolder = "./";
	    pIGPS_ = algo::AlgoGPS::createInstance();
	    poutput_ = ygomi::outPut::createInstance(debugFolder);
	    pTrsfTool_ = CommonAlgo::transformTools::createInstance();
    }
    

    
    Platform::~Platform()
    {
        std::cout << "dtor is called\n";
        m_newMapPointId.clear();
    }
    
    ygomi::Platform* Platform::createInstance()
    {
        if(!m_pInstance)
            m_pInstance = std::unique_ptr<Platform>(new Platform());
        return m_pInstance.get();
    }
    
    void Platform::releaseInstance()
    {
        if(m_pInstance){
            delete m_pInstance.release();
        }
        m_bInit = false;
    }
    
    bool Platform::processFrame(long frameId, std::string frameName){
        if(!setFrame(frameId, frameName.c_str()))
            return false;
        //========================================  track  ========================================
        extractFeatures(frameId);
        static ygomi::TrackingState state(ygomi::NOT_INITIALIZED);
        if(state == ygomi::NOT_INITIALIZED) {
            bool bOK = tryInitPose(frameId);
            if(bOK) {
                m_referKFID = frameId;
                state = ygomi::INITIALIZED;
            }
            std::cout << (bOK ? "init success\n" : "init failed\n");
        }
        else{
            bool bOK = false;
            bOK = trackWithMotionModel(frameId);
            decisionKeyFrame(frameId);
        }
        //========================================  mapping  ========================================
        if(!checkKF(frameId)) return true;
        int keyFrameCount = 10;
        searchByEpipolar(frameId, keyFrameCount);
        triangulate(frameId, keyFrameCount);
        return true;
    }
    
    bool Platform::init(const char* config, chamo::DataInterface* _dataInterface)
    {
        if(m_bInit)
            return m_bInit;
        
        try {
            dataInterface = _dataInterface;
            //init parameter parser.
            m_pParamParser = std::unique_ptr<ygomi::ParamParser>(new ygomi::ParamParser());
            CV_Assert(m_pParamParser);
            m_bInit = m_pParamParser->init(config);
            
            //init feature extractor.
            m_pFeatureExtractor = std::unique_ptr<ygomi::FeatureExtractor>(new ygomi::FeatureExtractor());
            CV_Assert(m_pFeatureExtractor);
            if(m_bInit && m_pFeatureExtractor) {
                m_bInit = m_pFeatureExtractor->init(m_pParamParser.get());
            }
            
            //defalt camera parameter
            float fx = m_pParamParser->parseFloat("fx");
            float fy = m_pParamParser->parseFloat("fy");
            float cx = m_pParamParser->parseFloat("cx");
            float cy = m_pParamParser->parseFloat("cy");
            m_K = cv::Mat::eye(3, 3, CV_32FC1);
            m_K.at<float>(0, 0) = fx;
            m_K.at<float>(1, 1) = fy;
            m_K.at<float>(0, 2) = cx;
            m_K.at<float>(1, 2) = cy;
            
            m_K.convertTo(m_K64f, CV_64FC1);
            
            m_DistCoef.create(4, 1, CV_32FC1);
            m_DistCoef.at<float>(0, 0) = m_pParamParser->parseFloat("k1");
            m_DistCoef.at<float>(1, 0) = m_pParamParser->parseFloat("k2");
            m_DistCoef.at<float>(2, 0) = m_pParamParser->parseFloat("p1");
            m_DistCoef.at<float>(3, 0) = m_pParamParser->parseFloat("p2");
            
            //init galobal map
            m_pGlobalMap = std::unique_ptr<ygomi::GlobalMap>(new ygomi::GlobalMap);
            
            viewerHandler = new chamo::ViewInterfaceImp();

            m_bInit = m_bInit && m_pGlobalMap;
        } catch(...) {
            std::cout << "Caught a general exception in initialization\n";
            m_bInit = false;
        }
        
        return m_bInit;
    }
    
    void Platform::changeCameraParam(float fx, float fy, float cx, float cy)
    {
        CV_Assert(m_pParamParser);
        
        m_K.at<float>(0, 0) = fx;
        m_K.at<float>(1, 1) = fy;
        m_K.at<float>(0, 2) = cx;
        m_K.at<float>(1, 2) = cy;
        
        m_K64f.at<double>(0, 0) = fx;
        m_K64f.at<double>(1, 1) = fy;
        m_K64f.at<double>(0, 2) = cx;
        m_K64f.at<double>(1, 2) = cy;
        
        m_pParamParser->addFloat("fx", fx);
        m_pParamParser->addFloat("fy", fy);
        m_pParamParser->addFloat("cx", cx);
        m_pParamParser->addFloat("cy", cy);
    }
    
    bool Platform::checkKF(long frameId)
    {
        return m_pGlobalMap->getFrame(frameId)->getType() == KEY_FRAME;
    }
    
    GlobalMap* Platform::getGMapPtr()
    {
        return m_pGlobalMap.get();
    }

    void GetCurrentOpenGLCameraMatrix(cv::Mat mCameraPose, pangolin::OpenGlMatrix &M)
    {

        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            {
                //unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
//                std::cout << "Rwc: " << Rwc << std::endl;
//                std::cout << "twc: " << twc << std::endl;
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();

//            std::cout << "M: " << M << std::endl;
    }

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, float mCameraSize, float mCameraLineWidth)
    {
        const float &w = mCameraSize;
        const float h = w*0.75;
        const float z = w*0.6;

        glPushMatrix();

        glMultMatrixd(Twc.m);

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();
        
        glPopMatrix();
    }

    void Platform::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                       float mKeyFrameSize, float mKeyFrameLineWidth, float mGraphLineWidth)
    {

//        bool bDrawGPS = false;

        const float &w = mKeyFrameSize;
        const float h = w*0.75;
        const float z = w*0.6;

        const std::vector<const ygomi::Frame*> vpKFs = m_pGlobalMap->getAllKeyFrames();

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                const ygomi::Frame*  pKF = vpKFs[i]; // vpKFs[i] is checked outside.
                cv::Mat Eye = cv::Mat::eye(4, 4, CV_64FC1);
                pKF->getPose().copyTo(Eye.rowRange(0, 3));
                cv::Mat Twc = Eye.inv().t();
                Twc.convertTo(Twc, CV_32F);

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }

//        if(bDrawGraph)
//        {
//            glLineWidth(mGraphLineWidth);
//            glColor4f(0.0f,1.0f,0.0f,0.6f);
//            glBegin(GL_LINES);
//
//            for(size_t i=0; i<vpKFs.size(); i++)
//            {
//                // vpKFs[i] is already checked before.
//                // Covisibility Graph
//                //                const vector<KFID > vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
//                long frameId = vpKFs[i]->getId();
//                int  frameCount = 1;
//                const std::vector<ygomi::Frame*> vCovKFs = m_pGlobalMap->getBestCovisibilityKeyFrames(frameId, frameCount);
//                const cv::Point3f& Ow = vpKFs[i]->getCameraCenter();
//                if(!vCovKFs.empty())
//                {
//                    for(std::vector<ygomi::Frame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
//                    {
//                        if(!(*vit) || ((*vit)<vpKFs[i]))
//                            continue;
//                        const cv::Point3f& Ow2 = (*vit)->getCameraCenter();
//                        glVertex3f(Ow.x, Ow.y,Ow.z);
//                        glVertex3f(Ow2.x, Ow2.y,Ow2.z);
//                    }
//                }
//
//                //                // Spanning tree
//                //                const KFID&  pParent = vpKFs[i]->GetParent();
//                //                if(pParent)
//                //                {
//                //                    const cv::Mat& Owp = pParent->GetCameraCenter();
//                //                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                //                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
//                //                }
//                //
//                //                // Loops
//                //                KeyFramePointSet sLoopKFs = vpKFs[i]->GetLoopEdges();
//                //                for(KeyFramePointSet::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
//                //                {
//                //                    if(!(*sit) || ((*sit)<vpKFs[i]))
//                //                        continue;
//                //                    const cv::Mat& Owl = (*sit)->GetCameraCenter();
//                //                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                //                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
//                //                }
//            }
//            
//            glEnd();
//        }

    }

    void Platform::DrawMapPoints(float mPointSize, long frameId)
    {
        std::vector<ygomi::MapPoint*> currFrameMPs;
        std::vector<ygomi::MapPoint*> vpMPs;
        currFrameMPs = m_pGlobalMap->getMapPointInFrame(frameId);
        vpMPs = m_pGlobalMap->getAllMapPoints();

        if(vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0, 0, 0);
        for(size_t i=0, iend=currFrameMPs.size(); i<iend;i++)
        {
            if(currFrameMPs[i]->isBad())
                continue;
            cv::Point3f pos = currFrameMPs[i]->getPosition();
            glVertex3f(pos.x, pos.y, pos.z);
        }

        glColor3f(255.f, 0, 0);

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad()) // vpMPs[i] is guarateed outside
                //if(vpMPs[i]->isBad() )
                continue;

            cv::Point3f pos = vpMPs[i]->getPosition();
            glVertex3f(pos.x, pos.y, pos.z);
        }
        glEnd();

        //        glPointSize(mPointSize);
        //        glBegin(GL_POINTS);
        //        glColor3f(1.0,0.0,0.0);
        //
        //
        //        for(MapPointPointSet::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        //        {
        //            if((*sit)->isBad())
        //                continue;
        //            cv::Mat pos = (*sit)->GetWorldPos();
        //            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        //
        //        }
        //
        //
        //        glEnd();
    }

    void Platform::showAll(long frameId)
    {
        // pose
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64FC1);
        // pose inverse
    	pangolin::OpenGlMatrix Twc;

        // get pose
        ygomi::Frame* pFrame = m_pGlobalMap->getFrame(frameId);
        if(pFrame->getType() != ygomi::KEY_FRAME)
            return;
        pFrame->getPose().copyTo(Tcw.rowRange(0, 3));
        Tcw.convertTo(Tcw, CV_32F);

//      std::cout << "Pose: " << pFrame->getPose() << std::endl;
//		std::cout << "Tcw: " << Tcw << std::endl;

        // parse params
        float mViewpointX = m_pParamParser->parseFloat("ViewpointX");
        float mViewpointY = m_pParamParser->parseFloat("ViewpointY");
        float mViewpointZ = m_pParamParser->parseFloat("ViewpointZ");
        float mViewpointF = m_pParamParser->parseFloat("ViewpointF");
        float mCameraSize      = m_pParamParser->parseFloat("CameraSize");
        float mCameraLineWidth = m_pParamParser->parseFloat("CameraLineWidth");
        float mKeyFrameSize     = m_pParamParser->parseFloat("KeyFrameSize");
        float mKeyFrameLineWidth = m_pParamParser->parseFloat("KeyFrameLineWidth");
        float mGraphLineWidth    = m_pParamParser->parseFloat("GraphLineWidth");
        float mPointSize         = m_pParamParser->parseFloat("PointSize");

       //=========================================
        if(initState == false)
        {
          // initial pangolin

            pangolin::CreateWindowAndBind("SLAM_MAP", 750, 500);

            // 3D Mouse handler requires depth testing to be enabled
            glEnable(GL_DEPTH_TEST);

            // Issue specific OpenGl we might need
            glEnable (GL_BLEND);
            glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // Define Camera Render Object (for view / scene browsing)
            ps_cam_ = new pangolin::OpenGlRenderState(
                    pangolin::ProjectionMatrix(800, 600, mViewpointF, mViewpointF,
                        400, 300, 0.1, 100000),
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ,
                        0, 0, 0, 0.0, -1.0, 0.0));

            // Add named OpenGL viewport to window and provide 3D Handler
            pd_disp_ = &pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, 0, 1.0, -400.0f/300.0f)
                                    .SetHandler(new pangolin::Handler3D(*ps_cam_));



            initState = true;
        }

        // draw map
        bool menuFollowCamera = true;
        bool menuShowPoints = true;
        bool menuShowKeyFrames = true;
        bool menuShowGraph = true;

        bool bFollow = true;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        GetCurrentOpenGLCameraMatrix(Tcw, Twc);
//        std::cout << "Tcw: " << Tcw << std::endl;

        if(menuFollowCamera && bFollow)
        {
            ps_cam_->Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            ps_cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            ps_cam_->Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        pd_disp_->Activate(*ps_cam_);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        DrawCurrentCamera(Twc, mCameraSize, mCameraLineWidth);

        if(menuShowKeyFrames || menuShowGraph)
			DrawKeyFrames(menuShowKeyFrames, menuShowGraph, mKeyFrameSize, mKeyFrameLineWidth, mGraphLineWidth);
        if(menuShowPoints)
            DrawMapPoints(mPointSize, frameId);


        // show track
        showTrack(frameId, "After Mapping", false);

        pangolin::FinishFrame();
    }

	
	void Platform::getCameraCenters(long begin, long end, std::vector<cv::Mat>& vCenter)
	{
		for (long i = begin; i < end; i++)
		{
			Frame* curFrame = m_pGlobalMap->getFrame(i);
			if(!curFrame)
			{
				std::cout<<"frame["<<i<<"] not found"<<std::endl;
				continue;
			}
			cv::Mat pose = curFrame->getPose().clone();
						
            cv::Mat R = pose.rowRange(0,3).colRange(0,3);
            cv::Mat t = pose.rowRange(0,3).col(3);
            cv::Mat center = -R.t()*t;
            vCenter.push_back(center);
		}
	}
	
	void Platform::getCameraPoses(long begin, long end, std::vector<cv::Mat>& vPose)
	{
		for (long i = begin; i < end; i++)
		{
			Frame* curFrame = m_pGlobalMap->getFrame(i);
			if(!curFrame)
			{
				std::cout<<"get camera pose frame["<<i<<"] not found"<<std::endl;
				continue;
			}
			cv::Mat pose = curFrame->getPose();
			
            vPose.push_back(pose);
			
		}
		return;
	}	
	void Platform::saveSlamGpsKML(long begin, long end, std::vector<cv::Mat>& vGps,std::vector<cv::Mat>& vCameraPose)
	{
		std::vector<cv::Mat> vCenter;
		getCameraCenters(begin, end, vCenter);
        pTrsfTool_->convertToAbsCamCenter(vCenter);
        poutput_->generateSlamKml("slam.kml", vCenter);
	    return;
	}

	
	bool Platform::computeTransform(long start, long end)
	{
		std::vector<cv::Mat> vCenter;
		std::vector<cv::Mat> vPose;
		std::vector<cv::Mat> vGps;
		getCameraCenters(start, end, vCenter);
		getCameraPoses(start, end, vPose);
		for (long i = start; i < end; i++)
		{
			cv::Point3f p3fGps;
			pIGPS_->getRelPos(i, p3fGps);
			cv::Mat matGps = cv::Mat::zeros(3,1,CV_64FC1);
			matGps.at<double>(0,0) = (double)p3fGps.x;
			matGps.at<double>(0,1) = (double)p3fGps.z;
			matGps.at<double>(0,2) = (double)p3fGps.y;
			//std::cout<<"frame "<<i<<"  p3fGps="<<p3fGps<<" matGps="<<matGps<<std::endl;
			vGps.push_back(matGps);
		}
		cv::Mat transform;
		if (!pTrsfTool_->computeSim3(vCenter, vGps, transform))
		{
			std::cout<<"plf compute transform failed!"<<std::endl;
		    return false;
		}
		
		std::cout<<"plf compute transform = "<<transform<<std::endl;
		pTrsfTool_->Slam2Gps(transform, vPose);
		for (long i = 0; i < end-start; i++)
		{
			Frame* curFrame = m_pGlobalMap->getFrame(start+i);
			if(!curFrame)
			{
				std::cout<<"frame["<<start+i<<"] not found"<<std::endl;
				continue;
			}
			
			curFrame->setPose(vPose[i]);
		}
		return true;
	}
}



