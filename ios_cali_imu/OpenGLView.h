//
//  OpenGLView.h
//  HelloGL
//
//  Created by wanglixin on 16/6/30.
//  Copyright © 2016年 Mac. All rights reserved.
//
#ifndef openglview_h
#define openglview_h
#import <UIKit/UIKit.h>

#import <QuartzCore/QuartzCore.h>
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>
#include <opencv2/opencv.hpp>
#include "common_header.h"

// Add to top of file
typedef struct {
    float Position[3];
    float Color[4];
} Vertex;

@interface OpenGLView : UIView
{
    double curTime;
    double lastTime;
    float fov;
    float dim;
    float asp;
    float ph;
    float th;
    float fixZ;
    cv::Mat tar;
    CAEAGLLayer* _eaglLayer;
    EAGLContext* _context;
    GLuint _colorRenderBuffer;
    
    GLuint _positionSlot;
    GLuint _colorSlot;
    
    GLuint _projectionUniform;
    
    GLuint _modelViewUniform;
    
    GLuint vertexBuffer;
    GLuint camBuffer;
    
    GLuint girdBuffer;
    
    float _currentRotation;
    
    GLuint _depthRenderBuffer;
    
    cv::Mat projMat;
    cv::Mat viewMat;
    std::vector<Vertex> Vertices;
    std::vector<Vertex> girdv;
    std::vector<Vertex> cams;
    
}
- (void) SetMapPoints: (std::vector<MapPointChamo>&) verts;
- (void) SetKF: (std::vector<cv::Mat>&) poses;
- (cv::Mat) getRotMat:(float)pitch yaw:(float) yaw roll:(float)roll;
- (cv::Mat) GetPoseFromRT: (cv::Mat) ori posi:(cv::Mat) posi;
- (void) AddKF: (cv::Mat) pose;
- (void) AddMapPoints: (MapPointChamo) vert;
- (void) render;
- (void) init_me;
@property BOOL isInited;
@end
#endif
