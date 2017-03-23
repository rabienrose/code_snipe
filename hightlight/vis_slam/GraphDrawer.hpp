/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file    GraphDrawer.h
 * @brief   draw camera, key frames, map points and covisibility.
 *
 * Change Log:
 *      Date             Who                      What
 *      2016.11.21       <Wei.Liu>                Created
 *******************************************************************************
 */

#pragma once

#ifndef _GRAPH_DRAWER_H_
#define _GRAPH_DRAWER_H_

// corresponding head file.

// system head file.
#include <string>
#include <list>
#include <cmath>

// lib or 3rdparty head file.
#include <opencv2/opencv.hpp>    // using some utilities of opencv
#include <pangolin/pangolin.h>    // using open gl libs.

// other local repository head file.

namespace algo_vehicle
{
    namespace visualizer
    {
        //==========================================================================
        // this struct are defined to organized the visualized data.

        // this flag will clear all buffered datas.
        typedef struct tagClear_t
        {
        }clear_t;

        typedef struct tagDraw_t
        {
        }draw_t;

        typedef struct tagBasicScale_t
        {
            tagBasicScale_t(float val) {val_ = val; }
            float val_;
        }basicScale_t;

        typedef struct tagScale_t
        {
            tagScale_t(float val) {val_ = val; }
            float val_;
        }scale_t;
        
        typedef struct tagColor_t
        {
            tagColor_t(const float x, const float y, const float z, const float a):val_(x, y, z, a){ }
            tagColor_t(const cv::Scalar & color) { val_ = color;}
            cv::Scalar val_;
        }color_t;

        typedef struct tagPtweight_t
        {
            tagPtweight_t(const float& weight) { val_ = weight; }
            float val_;
        }ptweight_t;

        typedef struct tagLnwidth_t
        {
            tagLnwidth_t(float val) {val_ = val; }
            float val_;
        }lnwidth_t;

        typedef struct tagVolsize_t
        {
            tagVolsize_t(float val) {val_ = val; }
            float val_;
        }volsize_t;

        typedef struct tagFollow_t
        {
            tagFollow_t(const cv::Mat& mat) {matrix_ = mat; }
            cv::Mat matrix_;
        }follow_t;
        
        typedef struct tagPoint_t
        {
            tagPoint_t(const float x, const float y, const float z):val_(x, y, z) { }
            tagPoint_t(const cv::Point3f& val):val_(val) { }
            
            static tagPoint_t Invalid() 
            {  return tagPoint_t(cv::Point3f(NAN, NAN, NAN)); }

            bool isInvalid() const
            {
                return std::isnan(val_.x) || std::isnan(val_.y) || std::isnan(val_.z);
            }
            cv::Point3f val_;
        }point_t;

        typedef struct tagLine_t
        {
            tagLine_t(const cv::Point3f& p1, const cv::Point3f& p2) { point1_ = p1; point2_ = p2; }
            static tagLine_t Invalid() 
            {  
                return tagLine_t(cv::Point3f(NAN, NAN, NAN), cv::Point3f(NAN, NAN, NAN)); 
            }
            bool isInvalid() const 
            {
                return std::isnan(point1_.x) || std::isnan(point1_.y) || std::isnan(point1_.z) || 
                       std::isnan(point2_.x) || std::isnan(point2_.y) || std::isnan(point2_.z);
            }
            cv::Point3f point1_;
            cv::Point3f point2_;
        }line_t;
        
        typedef struct tagTricone_t
        {
            tagTricone_t(cv::Mat mat, bool flag) { matrix_ = mat; isNeedDirection_ = flag; }
            static tagTricone_t Invalid() { return tagTricone_t(cv::Mat(), false); }
            bool isInvalid() const 
            {
                return matrix_.empty();
            }
            cv::Mat matrix_;
            bool    isNeedDirection_;
        }tricone_t;

        // this class will use stream input to draw graph using opengl.
        class GraphDrawer
        {
        public:
            // enums and typedefs
            template <typename FirstT, typename SecondT, typename ThirdT>
            struct tagTuple_t
            {
                tagTuple_t(const FirstT& first, const SecondT& second, const ThirdT& third):
                    first_(first), second_(second), third_(third)
                {
                }
                FirstT  first_;
                SecondT second_;
                ThirdT  third_;
            };

            template <typename FirstT, typename SecondT, typename ThirdT, typename FourthT>
            struct tagQuater_t
            {
                tagQuater_t(const FirstT& first, const SecondT& second, const ThirdT& third, const FourthT& fourth):
                    first_(first), second_(second), third_(third), fourth_(fourth)
                {
                }
                FirstT  first_;
                SecondT second_;
                ThirdT  third_;
                FourthT fourth_;
            };

        public:
            // ctors and dtors
            GraphDrawer();
            ~GraphDrawer();
            
            bool init(const std::string& winName);
            
        public:
            // public interfaces
            GraphDrawer& operator<< (const clear_t&);
            GraphDrawer& operator<< (const draw_t&);
            GraphDrawer& operator<< (const basicScale_t& scale);
            GraphDrawer& operator<< (const scale_t& scale);
            GraphDrawer& operator<< (const color_t& color);
            GraphDrawer& operator<< (const ptweight_t& weight);
            GraphDrawer& operator<< (const lnwidth_t& width);
            GraphDrawer& operator<< (const volsize_t& size);
            GraphDrawer& operator<< (const follow_t& follow);
            GraphDrawer& operator<< (const tricone_t& tricone);
            GraphDrawer& operator<< (const line_t& line);
            GraphDrawer& operator<< (const point_t& point);
        private:
            // private implementations.
            GraphDrawer& drawFollow ();
            GraphDrawer& drawTricones ();
            GraphDrawer& drawLines ();
            GraphDrawer& drawPoints ();
            void DrawGird();
        private:
            // buffers for drawing params, eg., color, point-weight, line-width, volume-size.  
            cv::Scalar color_;
            float       ptweight_;
            float       lnwidth_;
            float       volsize_;
            float       basicScale_;
            float       scale_;

        private:
            // buffers for data to display. eg., lines, points, volumes.
            typedef std::list< tagQuater_t<tricone_t, color_t, volsize_t, lnwidth_t> >  TriconeList;
            typedef std::list< tagTuple_t<line_t,    color_t, lnwidth_t> >  LineList;
            typedef std::list< tagTuple_t<point_t,   color_t, ptweight_t> > PointList;

            TriconeList tricones_;
            LineList    lines_;
            PointList   points_;
            
        private:
            // drawer instance
            pangolin::OpenGlRenderState* ps_cam_;
            pangolin::View* pd_disp_;
            pangolin::OpenGlMatrix mat_;

            pangolin::View* pd_img_;
            float mViewpointX = 0.0;
            float mViewpointY = 0.1;
            float mViewpointZ = 500.0;
            float mViewpointF = 20000;
            
        };

    }
} // namespace algo_vehicle::visualizer

#endif // fndef _GRAPH_DRAWER_H_

