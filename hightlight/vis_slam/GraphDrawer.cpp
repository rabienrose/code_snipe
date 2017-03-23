/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    GraphDrawer.cpp
 * @brief   draw camera, key frames, map points and covisibility.
 *
 * Change Log:
 *      Date             Who                      What
 *      2016.11.21       <Wei.Liu>                Created
 *******************************************************************************
 */

// corresponding head file.
#include "../visualisation/GraphDrawer.hpp"

// system head file.

// lib or 3rdparty head file.
#include <pangolin/pangolin.h>     // using pangolin to draw

// wrap whole visulization module into one namespace.
namespace algo_vehicle
{
    namespace visualizer
    {
        GraphDrawer::GraphDrawer():color_(cv::Scalar(1.0f, 0.0f, 0.0f, 1.0f)),
            ptweight_(2.0f), lnwidth_(2.0f), volsize_(0.1f), basicScale_(1.0f), scale_(1.0f)
        {
            tricones_.clear();
            lines_.clear();
            points_.clear();

            ps_cam_  = nullptr;
            pd_disp_ = nullptr;
            pd_img_  = nullptr;
        }

        GraphDrawer::~GraphDrawer()
        {
            tricones_.clear();
            lines_.clear();
            points_.clear();
            
            if(ps_cam_)
            {
                delete ps_cam_;
            }
            
            if(pd_disp_)
            {
                delete pd_disp_;
            }
            
            if(pd_img_)
            {
                delete pd_img_;
            }
        }

        bool GraphDrawer::init(const std::string& winName)
        {
            static bool first = true;
            if (first)
            {
                pangolin::CreateWindowAndBind(winName, 800, 600);

                // 3D Mouse handler requires depth testing to be enabled
                glEnable(GL_DEPTH_TEST);

                // Issue specific OpenGl we might need
                glEnable (GL_BLEND);
                glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                first = false;
            }

            // Define Camera Render Object (for view / scene browsing)
            ps_cam_ = new pangolin::OpenGlRenderState(
//                pangolin::ProjectionMatrixOrthographic(-8,8,-6,6, 0.1, 10000),
                pangolin::ProjectionMatrix(800, 600, mViewpointF, mViewpointF, 400, 300, 0.1, 100000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, -1, 0, 0, 0.0, 1.0, 0.0));

            // Add named OpenGL viewport to window and provide 3D Handler
            pd_disp_ = &pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -800.0f/600.0f)
                .SetHandler(new pangolin::Handler3D(*ps_cam_));

            // pd_img_ = &pangolin::Display("image")
            //   .SetBounds(0.5,1.0,0,0.5,-640.0/480)
            //   .SetLock(pangolin::LockLeft, pangolin::LockTop);

            mat_.SetIdentity();
            
            ps_cam_->Follow(mat_);

            return true;
        }

        // clear all buffered datas, do not need to clear drawing.
        GraphDrawer& GraphDrawer::operator<< (const clear_t&)
        {
            // clear opengl matrix.
            mat_.SetIdentity();

            // clear all buffered params.
            color_ = cv::Scalar(1.0f, 0.0f, 0.0f, 1.0f);
            ptweight_ = 2.0f;
            lnwidth_  = 2.0f;
            volsize_  = 0.1f;
            basicScale_ = 1.0f;
            scale_    = 1.0f;

            // clear all buffered datas
            tricones_.clear();
            lines_.clear();
            points_.clear();
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const draw_t&)
        {
            // step1: begin drawing, 
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // draw map
            bool menuFollowCamera = true;
            bool bFollow = true;
            
            if(menuFollowCamera && bFollow)
            {
                ps_cam_->Follow(mat_);
            }
            else if(menuFollowCamera && !bFollow)
            {
                ps_cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                ps_cam_->Follow(mat_);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            pd_disp_->Activate(*ps_cam_);
            glClearColor(1.0f,1.0f,1.0f,1.0f);

            DrawGird();
            // step2: draw concrete items.
            (void)drawTricones();
            (void)drawPoints();
            (void)drawLines();

            // step3: end drawing
            pangolin::FinishFrame();
            
            return *this;
        }
        
        void GraphDrawer::DrawGird()
        {
            int gird_step=10;
            int gird_range=100;
            glPushMatrix();
            glColor3f(0.0f,0.0f,0.0f);
            glLineWidth(1);
            for (int i=-gird_range; i<gird_range;i=i+gird_step){
                glBegin(GL_LINES);
                glVertex3f(-gird_range,i,0);
                glVertex3f(gird_range,i,0);
                glEnd();
            }
            
            for (int i=-gird_range; i<gird_range;i=i+gird_step){
                glBegin(GL_LINES);
                glVertex3f(i,gird_range,0);
                glVertex3f(i,-gird_range,0);
                glEnd();
            }
            glPopMatrix();
        }


        GraphDrawer& GraphDrawer::operator<< (const basicScale_t& scale)
        {
            if(0 == scale.val_)
            {
                return *this;
            }
            
            // this basic scale is only used to recover the size of tricone.
            basicScale_ = scale.val_;
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const scale_t& scale)
        {
            if(0 == scale.val_)
            {
                return *this;
            }
            
            // update mat_ first.
            mat_.m[12] *= (scale.val_/scale_);
            mat_.m[13] *= (scale.val_/scale_);
            mat_.m[14] *= (scale.val_/scale_);
            
            // update current graph scale.
            scale_ = scale.val_;
            
            return *this;
        }
        
        GraphDrawer& GraphDrawer::operator<< (const color_t& color)
        {
            color_ = color.val_;
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const ptweight_t& weight)
        {
            ptweight_ = weight.val_;
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const lnwidth_t& width)
        {
            lnwidth_ = width.val_;
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const volsize_t& size)
        {
            volsize_ = size.val_;
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const follow_t& follow)
        {
            // pose, do not need to convert to float, it MUST be guaranteed by outside
            cv::Mat camPose = follow.matrix_.clone();
            
            // pose inverse
            if(!camPose.empty())
            {
                cv::Mat Rwc(3,3,CV_32F);
                cv::Mat twc(3,1,CV_32F);
                {
                    Rwc = camPose.rowRange(0,3).colRange(0,3).t();
                    twc = -scale_ * Rwc*camPose.rowRange(0,3).col(3);
                }

//                mat_.m[0] = Rwc.at<float>(0,0);
//                mat_.m[1] = Rwc.at<float>(1,0);
//                mat_.m[2] = Rwc.at<float>(2,0);
//                mat_.m[3]  = 0.0;
//
//                mat_.m[4] = Rwc.at<float>(0,1);
//                mat_.m[5] = Rwc.at<float>(1,1);
//                mat_.m[6] = Rwc.at<float>(2,1);
//                mat_.m[7]  = 0.0;
//
//                mat_.m[8] = Rwc.at<float>(0,2);
//                mat_.m[9] = Rwc.at<float>(1,2);
//                mat_.m[10] = Rwc.at<float>(2,2);
//                mat_.m[11]  = 0.0;

                mat_.m[12] = twc.at<float>(0);
                mat_.m[13] = twc.at<float>(1);
                mat_.m[14] = twc.at<float>(2);
                mat_.m[15]  = 1.0;
            }
            else
            {
                mat_.SetIdentity();
            }

            return *this;
        }   

        GraphDrawer& GraphDrawer::operator<< (const tricone_t& tricone)
        {
            // if the input is invalid, then clear all related data
            if(tricone.isInvalid())
            {
                tricones_.clear();
                return *this;
            }

            // convert camera pose to opengl matrix.
            tricone_t newTricon(tricone);

            // extend the input 3*4 matrix to 4*4 matrix.
            cv::Mat eye = cv::Mat::eye(4, 4, CV_32F);
            tricone.matrix_.copyTo(eye.rowRange(0, 3));
            newTricon.matrix_= eye.inv().t();         // the t is used to convert to opengl matrix format.

            tricones_.push_back(tagQuater_t<tricone_t, color_t, volsize_t, lnwidth_t>(newTricon, color_, volsize_, lnwidth_));
            
            return *this;
        }   
        
        GraphDrawer& GraphDrawer::operator<< (const line_t& line)
        {
            // if the input is invalid, then clear all related data
            if(line.isInvalid())
            {
                lines_.clear();
                return *this;
            }

            // else, append data.
            lines_.push_back(tagTuple_t<line_t, color_t, lnwidth_t>(line, color_, lnwidth_));
            
            return *this;
        }

        GraphDrawer& GraphDrawer::operator<< (const point_t& point)
        {
            // if the input is invalid, then clear all related data
            if(point.isInvalid())
            {
                points_.clear();
                return *this;
            }

            // else, append data.
            points_.push_back(tagTuple_t<point_t, color_t, ptweight_t>(point, color_, ptweight_));
            
            return *this;
        }

        GraphDrawer& GraphDrawer::drawFollow ()
        {
            // the scale is computed into mat_, so DO NOT need to add scale for it.
            ps_cam_->Follow(mat_);

            return *this;
        }   

        GraphDrawer& GraphDrawer::drawTricones ()
        {
            bool isDebug = false;

            cv::Scalar colorRed(1.0f, 0.0f, 0.0f, 1.0f);
            cv::Scalar colorGreen(0.0f, 1.0f, 0.0f, 1.0f);
            cv::Scalar colorBlue(0.0f, 0.0f, 1.0f, 1.0f);
            
            // step1: check the input, it MUST be valid. 
            for(auto& elem: tricones_)
            {
                tricone_t tricone = elem.first_;
                color_t   stcolor = elem.second_;
                volsize_t volsize = elem.third_;
                lnwidth_t lnwidth = elem.fourth_;

                // begin drawing.
                cv::Mat Twc = tricone.matrix_;
                if(Twc.empty())
                {
                    return *this;
                }
                
                // step2: specify attributes for current drawing.
                cv::Scalar color = stcolor.val_;
                // const float w = volsize.val_ / basicScale_;
                const float w = volsize.val_;
                const float h = w*0.75;
                const float z = w*0.6;

                Twc.colRange(0, 3).row(3) *= scale_;
                Twc.at<float>(3, 3) = 1.0;
                
                glPushMatrix();
                
                glMultMatrixf(Twc.ptr<GLfloat>(0));
                
                glLineWidth(lnwidth.val_);
                
                // step3: draw this kf.
                glColor3f(color[0], color[1], color[2]);  // using cyan.
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
                
                // draw direction if needed.
                if(tricone.isNeedDirection_)
                {
                    // bgn: add by weiliu ,draw it's directions.
                    color = colorGreen;
                    glColor3f(color[0], color[1], color[2]);     // x: green.
                    glBegin(GL_LINES);
                    glVertex3f(0,0,0);
                    glVertex3f(3*w,0,0);
                    glEnd();
                
                    color = colorBlue;
                    glColor3f(color[0], color[1], color[2]);     // z: blue.
                    glBegin(GL_LINES);
                    glVertex3f(0,0,0);
                    glVertex3f(0,0,3*w);
                    glEnd();
                
                    color = colorRed;
                    glColor3f(color[0], color[1], color[2]);     // y: red.
                    glBegin(GL_LINES);
                    glVertex3f(0,0,0);
                    glVertex3f(0,3*w,0);
                    glEnd();
                }
                
                glPopMatrix();
            }
            
            return *this;
        }   

        GraphDrawer& GraphDrawer::drawLines ()
        {
            // step1: check the input, it MUST be valid. 
            for(auto& elem: lines_)
            {
                line_t    line    = elem.first_;
                color_t   stcolor = elem.second_;
                lnwidth_t lnwidth = elem.third_;

                // step3: draw this specified item.
                cv::Point3f pos1 = scale_ * line.point1_;
                cv::Point3f pos2 = scale_ * line.point2_;

                cv::Scalar color = stcolor.val_;
                glColor4f(color[0], color[1], color[2], color[3]);
                glLineWidth(lnwidth.val_);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pos1.x, pos1.y, pos1.z);
                glVertex3f(pos2.x, pos2.y, pos2.z);
                
                glEnd();
            }

            return *this;
        }


        GraphDrawer& GraphDrawer::drawPoints ()
        {
            // bool isDebug = true;
            
            // step1: check the input, it MUST be valid. 
            for(auto& elem: points_)
            {
                point_t    point  = elem.first_;
                color_t    stcolor  = elem.second_;
                ptweight_t weight = elem.third_;

                // step2: specify color for current drawing.
                
                // step3: draw this mappoint.
                cv::Scalar color = stcolor.val_;
                glPointSize(weight.val_);
                glBegin(GL_POINTS);
                glColor4f(color[0], color[1], color[2], color[3]);
                cv::Point3f pos = scale_ * point.val_;
                glVertex3f(pos.x, pos.y, pos.z);
                
                glEnd();
            }

            return *this;
        }

    }
}// namespace algo_vehicle::visualizer

