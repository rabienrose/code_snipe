
#include <mex.h>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Tracking.h"
#include "System.h"

static std::unique_ptr<ORB_SLAM2::Tracking> g_Tracker  = nullptr;
static std::unique_ptr<ORB_SLAM2::System> g_system     =  nullptr;

/**
 *
 *@param nrhs Input parameters number
 *@param prhs Input parameters array
 *
 */
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    assert(nrhs >= 1);
    const char* command = mxArrayToString(prhs[0]);
    
    mexPrintf("command: %s\n", command);
    
    if(!strcmp("init", command)) {
        assert(nrhs >= 3);
        const char* vocPath = mxArrayToString(prhs[1]);
        if(!vocPath) return;
        
        const char* configPath = mxArrayToString(prhs[2]);
        if(!configPath) return;
        
        //todo:
        //load camera parameters from Matlab, current read from path.
        
        int sensor = 0;//monocular
        //g_Tracker = std::unique_ptr<ORB_SLAM2::Tracking>(new Tracking(vocPath, configPath, 0));
        //g_Tracker->init(vocPath, configPath);
        const bool bUseViewer = false;
        g_system = std::unique_ptr<ORB_SLAM2::System>(new ORB_SLAM2::System(vocPath, configPath, ORB_SLAM2::System::MONOCULAR, bUseViewer));
        if(g_system) {
            mexPrintf("failed to init system");
        }
        return;
    }
    else if(!strcmp("track", command)) {
        assert(nrhs >= 3);
        //read image path
        const char* imagePath = mxArrayToString(prhs[1]);
        
        mexPrintf("image path: %s\n", imagePath);

        //read camera pose
        size_t s = mxGetN(prhs[2]);
        double* pose = mxGetPr(prhs[2]);
        float cameraPose[3][4];
        
        //mexPrintf("pose size: %d\n", s);

        for(int i=0 ; i<3 ; i++) {
            for(int j=0 ; j<4 ; j++) {
                cameraPose[i][j] = static_cast<float>(pose[i*4 + j]);
                mexPrintf("%f, ", cameraPose[i][j]);
            }
            mexPrintf("\n");
        }
         
        //read 3d map points.
        assert(nrhs >= 6);
        size_t numOfMapPoints = mxGetN(prhs[3]);
        assert(numOfMapPoints == mxGetN(prhs[4]));
        assert(numOfMapPoints == mxGetN(prhs[5]));

        const double* x_val = mxGetPr(prhs[3]);
        const double* y_val = mxGetPr(prhs[4]);
        const double* z_val = mxGetPr(prhs[5]);

        //mexPrintf("points num: %d\n", numOfMapPoints);
        
        std::vector<cv::Point3f> points3D;
        points3D.reserve(numOfMapPoints);
        for(size_t i=0 ; i<numOfMapPoints ; i++) {
            cv::Point3f pt;
            pt.x = static_cast<float>(x_val[i]);
            pt.y = static_cast<float>(y_val[i]);
            pt.z = static_cast<float>(z_val[i]);
            
            points3D.push_back(pt);
        }
        
        //mexPrintf("points num: %d(map points), %d\n", numOfMapPoints, points3D.size());
        
//         std::vector<cv::Point2f> points2D;
//         static bool firstInit = false;
//         if(firstInit) {
//             firstInit = false;
//             //read 2d points.
//             assert(nrhs >= 8);
//             assert(numOfMapPoints == mxGetN(prhs[6]));
//             assert(numOfMapPoints == mxGetN(prhs[7]));
//             const double* x_2d_val = mxGetPr(prhs[6]);
//             const double* y_2d_val = mxGetPr(prhs[7]);
// 
//             points2D.reserve(numOfMapPoints);
//             for(size_t i=0 ; i<numOfMapPoints ; i++) {
//                 points2D[i].x = static_cast<float>(x_2d_val[i]);
//                 points2D[i].y = static_cast<float>(y_2d_val[i]);
//             }        
//         }

        //do tracking
        if(g_Tracker) {
            //if(g_Tracker->track(imagePath, points3D, points2D, cameraPose) != 0)
        }
        if(g_system) {
            const std::string& path = "/Users/zhaiq/Downloads/1.jpg";
//             cv::Mat frame = cv::imread(imagePath);
//             if(frame.empty()) {
//                 mexPrintf("%sload image failed.\n", imagePath);
//                 return;
//             }
//             else {
//                 mexPrintf("load image success\n");
//             }
//             
//             mexPrintf("image info: %d, %d, %d", frame.cols, frame.rows, frame.channels());
            mexPrintf("tracking\n");
            
            double time_stamp = 0.0;
            g_system->TrackMonocular(std::string(imagePath), time_stamp);
        }

        //create a new array and save optimized camea pose.
        assert(nlhs >= 5);

        //save tracking status.
        plhs[0] = mxCreateDoubleMatrix((mwSize)1, (mwSize)1, mxREAL);
        double* status = mxGetPr(plhs[0]);
        status[0] = static_cast<double>(true);
        
        plhs[1] = mxCreateDoubleMatrix((mwSize)1, (mwSize)12, mxREAL);
        double* new_camera_pose = mxGetPr(plhs[0]);
        for(int i=0 ; i<3 ; i++) {
            for(int j=0 ; j<4 ; j++) {
                new_camera_pose[i*4 + j] = static_cast<double>(cameraPose[i][j]);
            }
        }
        
        //create new arrays, save updated map points.
        size_t updatedMapPointsNum = points3D.size();
        
        mexPrintf("points num : %d\n", updatedMapPointsNum);
        
        plhs[2] = mxCreateDoubleMatrix((mwSize)1, (mwSize)updatedMapPointsNum, mxREAL);
        plhs[3] = mxCreateDoubleMatrix((mwSize)1, (mwSize)updatedMapPointsNum, mxREAL);
        plhs[4] = mxCreateDoubleMatrix((mwSize)1, (mwSize)updatedMapPointsNum, mxREAL);

        double* new_x_val = mxGetPr(plhs[2]);
        double* new_y_val = mxGetPr(plhs[3]);
        double* new_z_val = mxGetPr(plhs[4]);
        for(size_t i=0 ; i<updatedMapPointsNum ; i++) {
            const cv::Point3f& pt = points3D[i];
            new_x_val[i] = static_cast<double>(pt.x);
            new_y_val[i] = static_cast<double>(pt.y);
            new_z_val[i] = static_cast<double>(pt.z);
        }
        mxFree((void*)imagePath);
    }
    mxFree((void*)command);
}