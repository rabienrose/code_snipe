#include "remoteConHelper.h"
#include "context.h"
#include <fstream>
#include <stdlib.h>


#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
namespace rviz_plugin_tutorials
{
std::string remoteConHelper::exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}
void remoteConHelper::testSSH()
{
    std::string remote_ip = LocContext::getInstance()->getParam("Control", "remote_ip");
    std::string account = LocContext::getInstance()->getParam("Control", "remote_account");
    std::string password = LocContext::getInstance()->getParam("Control", "remote_password");

    // check ssh connection
    std::stringstream ss;
    ss << "sshpass -p " << password << " ssh -q " << account << "@" << remote_ip << " exit ; echo $?";
    std::string result = exec(ss.str().c_str());
    std::cout<<"command: " << ss.str() << std::endl;
    std::cout<<"result: " << result << std::endl;
}
void remoteConHelper::parseEnv()
{
    std::string ROS_IP = getenv("ROS_IP");
    std::string ROS_MASTER_URI = getenv("ROS_MASTER_URI");

    if(!ROS_IP.empty() && !ROS_MASTER_URI.empty()) {
        // overwrite with params after run.sh
        std::cout << "ROS_IP: " << ROS_IP << " ROS_MASTER_URI: " << ROS_MASTER_URI << std::endl;

        LocContext::getInstance()->setParam("Control", "local_ip", ROS_IP);
        std::string REMOTE_IP = ROS_MASTER_URI.substr(7, ROS_MASTER_URI.find_last_of(':') - 7);
        std::cout << "remote ip: " << REMOTE_IP << std::endl;
        LocContext::getInstance()->setParam("Control", "remote_ip", REMOTE_IP);

        if (ROS_IP == "127.0.0.1" && ROS_MASTER_URI == "http://127.0.0.1:11311") {
            LocContext::getInstance()->setParam("Control", "switch_remote_control", "False");
        } else {
            char *user = getenv("REMOTE_ACCOUNT");
            char *psw = getenv("REMOTE_PASSWORD");
            if (!user && !psw) {
                // LocContext::getInstance()->setParam("Control", "switch_remote_control", "False");
            } else if (user && psw) {
                LocContext::getInstance()->setParam("Control", "switch_remote_control", "True");
                LocContext::getInstance()->setParam("Control", "remote_account", user);
                LocContext::getInstance()->setParam("Control", "remote_password", psw);
                std::cout << "remote account: " << user << "\tremote password: " << psw << std::endl;
            }
        }
    }

    std::cout << "receive config" << std::endl;

    std::string db_path = LocContext::getInstance()->getParam("Control", "database_path");
    std::string rosbag_path = LocContext::getInstance()->getParam("Control", "rosbag_path");
    std::string rtv_path = LocContext::getInstance()->getParam("Control", "rtv_path");
    bool do_remote_control = LocContext::getInstance()->getParam("Control", "switch_remote_control") == "True";

    std::stringstream ss;
    if(!db_path.empty()) {
        ss << "ls " << db_path << " -F | grep \"/$\"";
        // ss << "ls " << db_path;
        database_list = do_cmd(ss.str(), do_remote_control, true);
    }

    if(!rosbag_path.empty()) {
        ss.str("");
        ss << "";
        // ss << "ls -l " << rosbag_path << " | grep \"^-\"";
        // ss << "ls " << rosbag_path;
        ss << "find " << rosbag_path << " -type f \\( -name \"*.bag\" -o -name \"*.rosbag\" \\)";
        rosbag_list = do_cmd(ss.str(), do_remote_control, true);
    }

    std::stringstream ss1;
    ss1<<"export ROS_IP="<<LocContext::getInstance()->getParam("Control", "local_ip")
       <<" && export ROS_MASTER_URI=http://"<<LocContext::getInstance()->getParam("Control", "remote_ip")<<":11311/";
    set_ros_ip_str=ss1.str();
}
bool remoteConHelper::tryKillNode(std::string nodeName, bool isRemote, bool isBlock)
{
    std::vector<std::string> re_lines = do_cmd("rosnode list", isRemote, isBlock);
    for (int i = 0; i < re_lines.size(); i++)
    {
        if (re_lines[i] == nodeName)
        {
            std::stringstream ss;
            ss << "rosnode kill " << nodeName;
            re_lines = do_cmd(ss.str(), isRemote, isBlock);
            do_cmd("echo y | rosnode cleanup", isRemote, isBlock);
            return true;
        }
    }
    return false;
}
std::vector<std::string> remoteConHelper::split_(const std::string &s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (getline(ss, item, delim))
    {
        tokens.push_back(item);
    }
    return tokens;
}
QImage remoteConHelper::cvMat2QImage(const cv::Mat &mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for (int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for (int row = 0; row < mat.rows; row++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar *)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if (mat.type() == CV_8UC4)
    {
        //             qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar *)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        //             qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}
std::string remoteConHelper::intToStr(int i)
{
    std::stringstream ss;
    std::string s;
    ss << i;
    ss >> s;
    return s;
}
std::string remoteConHelper::doubleToStr(double i)
{
    std::stringstream ss;
    std::string s;
    ss << i;
    ss >> s;
    return s;
}

std::string remoteConHelper::dirName(const std::string &path)
{
    std::string dir = ".";
    std::size_t pos = path.rfind("/");
    if (pos != std::string::npos)
    {
        dir = path.substr(0, pos);
    }
    return dir;
}

std::string remoteConHelper::fileName(const std::string &path)
{
    std::string dir = path;
    if(dir.back() == '/')
        dir.pop_back();
    std::size_t pos = dir.rfind("/");
    if (pos != std::string::npos)
    {
        return dir.substr(pos+1);
    }
    else
        return  dir;
}

std::vector<std::string> remoteConHelper::do_cmd(std::string do_what, bool isRemote, bool isBlock)
{
    std::stringstream ss;
    std::cout << "cmd: " << do_what << std::endl;
    std::string psw = LocContext::getInstance()->getParam("Control", "remote_password");
    std::string user = LocContext::getInstance()->getParam("Control", "remote_account");
    std::string ip = LocContext::getInstance()->getParam("Control", "remote_ip");
    if (isRemote && "True" == LocContext::getInstance()->getParam("Control", "switch_remote_control"))
    {
        if (isBlock)
        {
            ss << "sshpass -p " << psw << " ssh " << user << "@" << ip << " '" << do_what << "'> /tmp/chamo.txt 2>&1";
        }
        else
        {
            ss << "sshpass -p " << psw << " ssh " << user << "@" << ip << " '" << do_what << "'&";
        }
    }
    else
    {
        if (isBlock)
        {
            ss << do_what << "> /tmp/chamo.txt 2>&1";
        }
        else
        {
            ss << do_what << "&";
        }
    }
    system(ss.str().c_str());

    std::ifstream temp_file;
    temp_file.open("/tmp/chamo.txt");
    std::string text_re;
    std::vector<std::string> text_re_vec;
    int line_count = 0;
    while (getline(temp_file, text_re))
    {
        if(text_re.empty() || (text_re[0]==' ' || text_re[0] == '\0' || text_re[0] == '\n'))
            continue;
        line_count++;
        text_re_vec.push_back(text_re);
        // std::cout << "line " << line_count << ":\t" << text_re << std::endl;
    }
    temp_file.close();

    return text_re_vec;
}
} // end namespace rviz_plugin_tutorials