#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace ORB_SLAM2
{
    class Optimizer
    {
    public:
       static int PoseOptimization(
                                    const std::vector<float>& cameraParam,
                                    const cv::Mat& initialPose,
                                    const std::vector<cv::Point3f>& mapPoints,
                                    const std::vector<cv::KeyPoint>& keysIn2D,
                                    const std::vector<float>& mvInvLevelSigma2,
                                    cv::Mat& optimizedPose,
                                    std::vector<bool>& bOutliers);

    };

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
