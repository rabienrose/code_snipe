#ifndef img_tool_h
#define img_tool_h

#include <vector>
#include <opencv2/opencv.hpp>
class Settings
{
public:
    Settings(){
        calibFixPrincipalPoint = false;
        calibZeroTangentDist = false;
        aspectRatio =0;
    }
    cv::Size img_size;
    cv::Size boardSize;              // The size of the board -> Number of items by width and height
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    float aspectRatio;           // The aspect ratio
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    int flag;
    void init(){
        flag = cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;
        if(calibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |=cv::CALIB_FIX_ASPECT_RATIO;
    }
};

class img_tool{
public:
    static bool runCalibration( Settings& s, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                               std::vector<std::vector<cv::Point2f> > imagePoints,
                               std::vector<cv::Mat>& rvecs,
                               std::vector<cv::Mat>& tvecs,
                               std::vector<float>& reprojErrs,  double& totalAvgErr,
                               std::vector<cv::Point3f>& mps);
    cv::Mat cal_corners(cv::Mat img, cv::Size board_size, std::vector<cv::Point2f>& img_points);
};
#endif /* customView_h */
