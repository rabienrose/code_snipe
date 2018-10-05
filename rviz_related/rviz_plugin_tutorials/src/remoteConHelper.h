#ifndef REMOTE_CON_HELPER_H
#define REMOTE_CON_HELPER_H

#include <opencv2/opencv.hpp> // using some utilities of opencv

#include <QImage>
namespace rviz_plugin_tutorials
{
class remoteConHelper
{
  public:
    static remoteConHelper *getInst()
    {
        static remoteConHelper *me = NULL;
        if (me == NULL)
        {
            me = new remoteConHelper();
        }
        return me;
    }

    std::vector<std::string> do_cmd(std::string do_what, bool isRemote = false, bool isBlock = true);
    void parseEnv();
    bool tryKillNode(std::string nodeName, bool isRemote = true, bool isBlock = true);
    std::string set_ros_ip_str;
    std::vector<std::string> database_list;
    std::vector<std::string> rosbag_list;
    std::vector<std::string> rtv_list;
    std::vector<std::string> split_(const std::string &s, char delim);
    QImage cvMat2QImage(const cv::Mat &mat);
    std::string intToStr(int i);
    std::string doubleToStr(double i);
    std::string dirName(const std::string &path);
    std::string fileName(const std::string &path);
    bool enableRemoteControl=false;
    void testSSH();
    std::string exec(const char* cmd);

  private:
    remoteConHelper()
    {
        std::cout << "get config" << std::endl;
        parseEnv();
    };
};
} // end namespace rviz_plugin_tutorials
#endif
