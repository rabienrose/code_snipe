//
//  OpenGLView.m
//  HelloGL
//
//  Created by wanglixin on 16/6/30.
//  Copyright © 2016年 Mac. All rights reserved.
//

#import "OpenGLView.h"
#include <math.h>
#include "Eigen/eigen"
#include <opencv2/core/eigen.hpp>
#define Cos(th) cos(3.1415926/180*(th))
#define Sin(th) sin(3.1415926/180*(th))
#define Tan(x) tan(3.1415926/180*(x))

@implementation OpenGLView
@synthesize isInited;

+ (Class)layerClass {
    return [CAEAGLLayer class];
}

- (void)setupLayer {
    _eaglLayer = (CAEAGLLayer*) self.layer;
    _eaglLayer.opaque = YES;
}

- (void)setupContext {
    EAGLRenderingAPI api = kEAGLRenderingAPIOpenGLES2;
    _context = [[EAGLContext alloc] initWithAPI:api];
    if (!_context) {
        NSLog(@"Failed to initialize OpenGLES 2.0 context");
        exit(1);
    }
    
    if (![EAGLContext setCurrentContext:_context]) {
        NSLog(@"Failed to set current OpenGL context");
        exit(1);
    }
}

- (cv::Mat)getRotMat:(float)pitch yaw:(float)yaw roll:(float)roll{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    cv::Mat re;
    eigen2cv(rotationMatrix, re);
    re.convertTo(re, CV_32FC1);
    return re;
}

- (void)setupRenderBuffer {
    glGenRenderbuffers(1, &_colorRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _colorRenderBuffer);
    [_context renderbufferStorage:GL_RENDERBUFFER fromDrawable:_eaglLayer];
}

- (void)setupFrameBuffer {
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _colorRenderBuffer);
    
    
    // Add to end of setupFrameBuffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _depthRenderBuffer);
}


// Add new method before init
- (void)setupDisplayLink {
    CADisplayLink *displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(render)];
    [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

// Add new method right after setupRenderBuffer
- (void)setupDepthBuffer {
    glGenRenderbuffers(1, &_depthRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _depthRenderBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, self.frame.size.width, self.frame.size.height);
}

cv::Mat populateFromFrustumLeft(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat near, GLfloat far) {
    cv::Mat projMat_t(4,4,CV_32FC1);
    
    projMat_t.at<float>(0,0)  = (2.0 * near) / (right - left);
    projMat_t.at<float>(1,0)  = 0.0;
    projMat_t.at<float>(2,0)  = 0.0;
    projMat_t.at<float>(3,0) = 0.0;
    
    projMat_t.at<float>(0,1)  = 0.0;
    projMat_t.at<float>(1,1)  = (2.0 * near) / (top - bottom);
    projMat_t.at<float>(2,1)  = 0.0;
    projMat_t.at<float>(3,1) = 0.0;
    
    projMat_t.at<float>(0,2)  = (right + left) / (right - left);
    projMat_t.at<float>(1,2)  = (top + bottom) / (top - bottom);
    projMat_t.at<float>(2,2) = -(far + near) / (far - near);
    projMat_t.at<float>(3,2) = -1.0;
    
    projMat_t.at<float>(0,3)  = 0.0;
    projMat_t.at<float>(1,3)  = 0.0;
    projMat_t.at<float>(2,3) = -(2.0 * far * near) / (far - near);
    projMat_t.at<float>(3,3) = 0.0;
    return projMat_t;
}

cv::Mat LookAt(cv::Mat posi, cv::Mat tar, cv::Mat up) {
    cv::Mat dir =tar - posi;
    cv::Mat z =dir;
    cv::normalize(z, z);
    cv::Mat x =  up.cross(z); // x = up cross z
    cv::normalize(x, x);
    cv::Mat y= z.cross(x); // y = z cross x
    cv::Mat re = cv::Mat::eye(3,3,CV_32FC1);
    x.copyTo(re.col(0).rowRange(0, 3));
    y.copyTo(re.col(1).rowRange(0, 3));
    z.copyTo(re.col(2).rowRange(0, 3));
    
    return re;
}

- (void) Projection
{
    float zn = dim / 4;
    float zf = dim * 200;
    float nearPlanH = zn*Tan(fov / 2);
    projMat = populateFromFrustumLeft(-nearPlanH*asp*dim, nearPlanH*asp*dim, -nearPlanH*dim, nearPlanH*dim, zn, zf).t();
}
- (id)init{
    std::cout<<"init called"<<std::endl;
    isInited=false;
    return self;
}
- (void) SetViewMat
{
    cv::Mat posi(3,1,CV_32FC1);
    posi.at<float>(0) = Cos(ph)*Sin(th)*fixZ;
    posi.at<float>(1) = Sin(ph)*fixZ;
    posi.at<float>(2) = Cos(ph)*Cos(th)*fixZ;
    cv::Mat up = cv::Mat::zeros(3, 1, CV_32FC1);
    up.at<float>(1) = 1;
    viewMat = cv::Mat::eye(4, 4, CV_32FC1);
    cv::Mat rMat = LookAt(posi, cv::Mat::zeros(3, 1, CV_32FC1), up);
    rMat.copyTo(viewMat.colRange(0, 3).rowRange(0, 3));
    viewMat.at<float>(2,3) =-fixZ;
    cv::Mat viewMatInv = viewMat.inv();
    viewMatInv.at<float>(0,3) = viewMatInv.at<float>(0,3) + tar.at<float>(0,0);
    viewMatInv.at<float>(1,3) = viewMatInv.at<float>(1,3) + tar.at<float>(1,0);
    viewMatInv.at<float>(2,3) = viewMatInv.at<float>(2,3) + tar.at<float>(2,0);
    viewMat=viewMatInv.inv();
    viewMat=viewMat.t();
    //std::cout<<viewMat<<std::endl;
}

- (void) mouseMoveEvent: (UIGestureRecognizer *)sender
{
    UIPanGestureRecognizer* panReco = (UIPanGestureRecognizer* )sender;
    CGPoint touchPoint = [panReco velocityInView:self];
    tar.at<float>(0,0) = tar.at<float>(0,0) - touchPoint.x/8000;
    tar.at<float>(2,0) = tar.at<float>(2,0) + touchPoint.y/8000;
    //NSLog(@"Pan:%f,%f",touchPoint.x,touchPoint.y);
    [self SetViewMat];
    [self render];
}

- (void) mouseMoveEvent1: (UIGestureRecognizer *)sender
{
    UIPanGestureRecognizer* panReco = (UIPanGestureRecognizer* )sender;
    CGPoint touchPoint = [panReco velocityInView:self];
    //NSLog(@"Pan:%f,%f",touchPoint.x,touchPoint.y);
    th = (int)(th + touchPoint.x/100) % 360;      //  Translate x movement to azimuth
    if (touchPoint.y/100<0){
        if(ph>-200){
            ph = (int)(ph + touchPoint.y/100);
        }
    }
    if (touchPoint.y/100>0){
        if(ph<-90){
            ph = (int)(ph + touchPoint.y/100);
        }
    }
    [self SetViewMat];
    [self render];
}

- (void) wheelEvent: (UIGestureRecognizer *)sender
{
    UIPinchGestureRecognizer* pinchReco = (UIPinchGestureRecognizer* )sender;
    //NSLog(@"Pinch:%f",pinchReco.velocity);
    if (pinchReco.velocity > 0)
        dim -= 0.01;
    else
        dim += 0.01;
    [self Projection];
    [self render];
}


- (void)render {
    [self updateVBOs];
    [self renderGird];
    [self renderObj];
    [self renderCam];
    [_context presentRenderbuffer:GL_RENDERBUFFER];
}

- (void)renderObj {
    if(Vertices.size()<=0){
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, Vertices.size()*sizeof(Vertex), Vertices.data(), GL_STATIC_DRAW);
    glUniformMatrix4fv(_projectionUniform, 1, 0, (GLfloat*)projMat.data);
    glUniformMatrix4fv(_modelViewUniform, 1, 0, (GLfloat*)viewMat.data);
    glViewport(0, 0, self.frame.size.width, self.frame.size.height);
    glVertexAttribPointer(_positionSlot, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer(_colorSlot, 4, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) *3));
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDrawArrays(GL_POINTS, 0, Vertices.size());
}

- (void)renderCam {
    if(cams.size()<=0){
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, camBuffer);
    glBufferData(GL_ARRAY_BUFFER, cams.size()*sizeof(Vertex), cams.data(), GL_STATIC_DRAW);
    glUniformMatrix4fv(_projectionUniform, 1, 0, (GLfloat*)projMat.data);
    glUniformMatrix4fv(_modelViewUniform, 1, 0, (GLfloat*)viewMat.data);
    glViewport(0, 0, self.frame.size.width, self.frame.size.height);
    glVertexAttribPointer(_positionSlot, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer(_colorSlot, 4, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) *3));
    glDrawArrays(GL_LINES, 0, cams.size());
}


- (void)renderGird {
    if(girdv.size()<=0){
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, girdBuffer);
    glBufferData(GL_ARRAY_BUFFER, girdv.size()*sizeof(Vertex), girdv.data(), GL_STATIC_DRAW);
    glUniformMatrix4fv(_projectionUniform, 1, 0, (GLfloat*)projMat.data);
    glUniformMatrix4fv(_modelViewUniform, 1, 0, (GLfloat*)viewMat.data);
    glViewport(0, 0, self.frame.size.width, self.frame.size.height);
    glVertexAttribPointer(_positionSlot, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer(_colorSlot, 4, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) *3));
    glDrawArrays(GL_LINES, 0, girdv.size());
}

- (GLuint)compileShader:(NSString*)shaderName withType:(GLenum)shaderType {
    // 1
    NSString* shaderPath = [[NSBundle mainBundle] pathForResource:shaderName
                                                           ofType:@"glsl"];
    NSError* error;
    NSString* shaderString = [NSString stringWithContentsOfFile:shaderPath
                                                       encoding:NSUTF8StringEncoding error:&error];
    if (!shaderString) {
        NSLog(@"Error loading shader: %@", error.localizedDescription);
        exit(1);
    }
    
    // 2
    GLuint shaderHandle = glCreateShader(shaderType);
    
    // 3
    const char* shaderStringUTF8 = [shaderString UTF8String];
    int shaderStringLength = [shaderString length];
    glShaderSource(shaderHandle, 1, &shaderStringUTF8, &shaderStringLength);
    
    // 4
    glCompileShader(shaderHandle);
    
    // 5
    GLint compileSuccess;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compileSuccess);
    if (compileSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetShaderInfoLog(shaderHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString); 
        exit(1); 
    } 
    
    return shaderHandle;
}

- (void)compileShaders {
    // 1
    GLuint vertexShader = [self compileShader:@"SimpleVertex"
                                     withType:GL_VERTEX_SHADER];
    GLuint fragmentShader = [self compileShader:@"SimpleFragment"
                                       withType:GL_FRAGMENT_SHADER];
    
    // 2
    GLuint programHandle = glCreateProgram();
    glAttachShader(programHandle, vertexShader);
    glAttachShader(programHandle, fragmentShader);
    glLinkProgram(programHandle);
    
    // 3
    GLint linkSuccess;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
    if (linkSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetProgramInfoLog(programHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    // 4
    glUseProgram(programHandle);
    
    // 5
    _positionSlot = glGetAttribLocation(programHandle, "Position");
    _colorSlot = glGetAttribLocation(programHandle, "SourceColor");
    glEnableVertexAttribArray(_positionSlot);
    glEnableVertexAttribArray(_colorSlot);
    
    // Add to bottom of compileShaders
    _projectionUniform = glGetUniformLocation(programHandle, "Projection");
    
    // Add to end of compileShaders
    _modelViewUniform = glGetUniformLocation(programHandle, "Modelview");
}

void addLine(cv::Point3f p1, cv::Point3f p2, std::vector<Vertex>& buffer, int color){
    float tColor[4];
    if(color==1){ //black
        tColor[0] =0;
        tColor[1] =0;
        tColor[2] =0;
        tColor[3] =1;
    }else if(color ==2){ //red
        tColor[0] =1;
        tColor[1] =0;
        tColor[2] =0;
        tColor[3] =1;
    }
    Vertex v1;
    v1.Position[0] = p1.x;
    v1.Position[1] = p1.y;
    v1.Position[2] = p1.z;
    v1.Color[0] =tColor[0];
    v1.Color[1] =tColor[1];
    v1.Color[2] =tColor[2];
    v1.Color[3] =tColor[3];
    buffer.push_back(v1);
    Vertex v2;
    v2.Position[0] = p2.x;
    v2.Position[1] = p2.y;
    v2.Position[2] = p2.z;
    v2.Color[0] =tColor[0];
    v2.Color[1] =tColor[1];
    v2.Color[2] =tColor[2];
    v2.Color[3] =tColor[3];
    buffer.push_back(v2);
}

- (void)setupVBOs {
    glGenBuffers(1, &vertexBuffer);
    glGenBuffers(1, &girdBuffer);
    glGenBuffers(1, &camBuffer);
    float grid_size = 10;
    float gird_step =1;
    for (float j = -grid_size; j <= grid_size; j=j+gird_step){
        addLine(cv::Point3f(-grid_size, 0,j), cv::Point3f(grid_size, 0,j), girdv, 1);
    }
    for (float j = -grid_size; j <= grid_size; j=j+gird_step){
        addLine(cv::Point3f(j, 0,-grid_size), cv::Point3f(j, 0,grid_size), girdv, 1);
    }
}

- (void)updateVBOs {
    glClearColor(1, 1, 1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glEnable(GL_DEPTH_TEST);
}


// Replace initWithFrame with this
- (void)init_me
{
    CGRect frame = self.frame;
    NSLog(@"glview loaded");
    asp = frame.size.width / frame.size.height;
    dim = 1;
    th = 180;
    ph = -90;
    fixZ = 10;
    fov =60;
    tar= cv::Mat::zeros(3, 1, CV_32FC1);
    UIPanGestureRecognizer *panReco = [[UIPanGestureRecognizer alloc]
                                       initWithTarget:self
                                       action:@selector(mouseMoveEvent:)];
    panReco.maximumNumberOfTouches =1;
    [self addGestureRecognizer:panReco];
    UIPanGestureRecognizer *rotReco = [[UIPanGestureRecognizer alloc]
                                       initWithTarget:self
                                       action:@selector(mouseMoveEvent1:)];
    rotReco.minimumNumberOfTouches =2;
    [self addGestureRecognizer:rotReco];
    UIPinchGestureRecognizer *pinchReco = [[UIPinchGestureRecognizer alloc]
                                           initWithTarget:self
                                           action:@selector(wheelEvent:)];
    [self addGestureRecognizer:pinchReco];
    [self setupLayer];
    [self setupContext];
    
    // Add to initWithFrame, right before call to setupRenderBuffer
    [self setupDepthBuffer];
    
    [self setupRenderBuffer];
    [self setupFrameBuffer];
    
    [self compileShaders];
    [self setupVBOs];
    //[self render];
        //[self setupDisplayLink];
    [self Projection];
    [self SetViewMat];
    isInited=true;
}

- (void) AddMapPoints: (MapPointChamo) vert{
    Vertex v;
    v.Position[0] = vert.posi.x;
    v.Position[1] = vert.posi.y;
    v.Position[2] = vert.posi.z;
    v.Color[0] =0;
    v.Color[1] =0;
    v.Color[2] =1;
    v.Color[3] =0.3;
    Vertices.push_back(v);
}

- (void) SetMapPoints: (std::vector<MapPointChamo>&) verts{
    Vertices.clear();
    for (int i=0; i<verts.size();i++){
        Vertex v;
        v.Position[0] = verts[i].posi.x;
        v.Position[1] = verts[i].posi.y;
        v.Position[2] = verts[i].posi.z;
        v.Color[0] =0;
        v.Color[1] =0;
        v.Color[2] =1;
        v.Color[3] =0.3;
        Vertices.push_back(v);
    }
}

cv::Point3f transPosi(cv::Point3f p, cv::Mat pose){
    cv::Mat posiM(4,1,CV_32FC1);
    posiM.at<float>(0,0)=p.x;
    posiM.at<float>(1,0)=p.y;
    posiM.at<float>(2,0)=p.z;
    posiM.at<float>(3,0)=1;
    cv::Mat poseInv=pose.inv();
    cv::Mat transdPosi=poseInv*posiM;
    cv::Point3f rePosi;
    rePosi.x =transdPosi.at<float>(0,0);
    rePosi.y =transdPosi.at<float>(1,0);
    rePosi.z =transdPosi.at<float>(2,0);
    return rePosi;
}

-(void) addCam: (cv::Mat) pose{
    float w = 0.1f;
    float h = w*0.75f;
    float z = w*0.6f;
    cv::Point3f p1 = transPosi(cv::Point3f(0, 0, 0), pose);
    cv::Point3f p2 = transPosi(cv::Point3f(w,h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(0, 0, 0), pose);
    p2 = transPosi(cv::Point3f(w,-h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(0, 0, 0), pose);
    p2 = transPosi(cv::Point3f(-w,-h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(0, 0, 0), pose);
    p2 = transPosi(cv::Point3f(-w,h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(w,h,z), pose);
    p2 = transPosi(cv::Point3f(w,-h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(-w,h,z), pose);
    p2 = transPosi(cv::Point3f(-w,-h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(-w,h,z), pose);
    p2 = transPosi(cv::Point3f(w,h,z), pose);
    addLine(p1, p2, cams, 2);
    
    p1 = transPosi(cv::Point3f(-w,-h,z), pose);
    p2 = transPosi(cv::Point3f(w,-h,z), pose);
    addLine(p1, p2, cams, 2);
}

- (void) SetKF: (std::vector<cv::Mat>&) poses{
    cams.clear();
    for (int i=0; i<poses.size();i++){
        [self addCam:poses[i]];
    }
}

- (cv::Mat) GetPoseFromRT: (cv::Mat) ori posi:(cv::Mat) posi{
    cv::Mat re =cv::Mat::eye(4, 4, CV_32F);
    ori.copyTo(re.rowRange(0, 3).colRange(0, 3));
    posi.copyTo(re.rowRange(0, 3).col(3));
    //re = re.inv();
    return re;
}

- (void) AddKF: (cv::Mat) pose{
    [self addCam:pose];
}

// Replace dealloc method with this
- (void)dealloc
{
//    [_context release];
    _context = nil;
//    [super dealloc];
}


@end
