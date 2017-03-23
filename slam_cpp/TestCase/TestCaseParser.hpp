
#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <opencv2/core.hpp>

namespace ygomi
{
    class TestCaseParser
    {
    public:
        TestCaseParser(std::string _xmlDir, std::string _rootDir);
        std::vector<std::string> GetCaseNames();
        std::string GetImagDir(std::string caseName, int frameId);
        int GetStartFrame(std::string caseName);
        int GetEndFrame(std::string caseName);
        void GetCamConfig(std::string caseName, float& fx, float& fy, float& cx, float& cy);
        std::string GetConfigDir(std::string caseName);
        template<typename T>
        T GetTestNode(std::string rootName, std::string caseName, std::string param);
    protected:
        std::string rootImgDir;
        std::string xmlDir;
    };
}