#ifndef POSE_VOTING
#define POSE_VOTING
#include "opencv2/core.hpp"
#include <unordered_map>
#include <vector>
#include <functional>
#include <set>
typedef cv::Point2i cell_t;
namespace std
{
    template<> struct hash<cell_t>
    {
        typedef cell_t argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const& s) const
        {
            result_type const h1 ( std::hash<int>{}(s.x) );
            result_type const h2 ( std::hash<int>{}(s.y) );
            return h1 ^ (h2 << 1); // or use boost::hash_combine
        }
    };
}



class VotingSpace{
public:

    void getDepthRange(cv::Mat K, cv::Point3f posi, cv::Point2f uv, float pitchAng, float eer_h, float cam_h, float eer_r, float &minD, float &maxD);
    void addErrShape(cv::Point3f, float minD, float maxD, int mpid);
    bool checkRange(cv::Mat pose);
    cv::Mat visVotingSpace(std::set<int>& candi_pairs);
    cv::Point2f gps;
    std::unordered_map<cell_t, std::vector<int>> votingSpace;

};
#endif
