#include "img_tool.h"

static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners){
    corners.clear();
    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
}

static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                        const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                        const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                        const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                        std::vector<float>& perViewErrors){
    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    for(size_t i = 0; i < objectPoints.size(); ++i ){
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);
        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }
    return std::sqrt(totalErr/totalPoints);
}

bool img_tool::runCalibration( Settings& s, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                           std::vector<std::vector<cv::Point2f> > imagePoints,
                           std::vector<cv::Mat>& rvecs,
                           std::vector<cv::Mat>& tvecs,
                           std::vector<float>& reprojErrs,  double& totalAvgErr,
                              std::vector<cv::Point3f>& mps){
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if( s.flag & cv::CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);
    mps =objectPoints[0];
    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    double rms;
    rms = cv::calibrateCamera(objectPoints, imagePoints, s.img_size, cameraMatrix, distCoeffs, rvecs, tvecs, s.flag);
    std::cout << "Re-projection error reported by calibrateCamera: "<< rms << std::endl;
    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
    return ok;
}

cv::Mat img_tool::cal_corners(cv::Mat img, cv::Size board_size, std::vector<cv::Point2f>& img_points){
    if(img.channels()>1){
        cv::cvtColor(img, img, CV_BGR2GRAY);
    }
    cv::Size patternsize= board_size;
    cv::Mat gray = img;
    bool patternfound = cv::findChessboardCorners(gray, patternsize, img_points,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                              + cv::CALIB_CB_FAST_CHECK);
    
    if(patternfound){
        cornerSubPix(gray, img_points, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    //std::cout<<"detect count: "<<corners.size()<<std::endl;
    cv::cvtColor(img, img, CV_GRAY2BGRA);
    cv::drawChessboardCorners(img, patternsize, cv::Mat(img_points), patternfound);
    return img;
}
