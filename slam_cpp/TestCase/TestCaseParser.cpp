#include "TestCaseParser.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <cassert>
#include <sstream>
#include <iomanip>

#define TESTCASE "TestCase"
#define CAMERA "Camera"

namespace ygomi
{
    TestCaseParser::TestCaseParser(std::string _xmlDir, std::string _rootImgDir){
        xmlDir= _xmlDir;
        rootImgDir = _rootImgDir;
    }
    
    std::string TestCaseParser::GetImagDir(std::string caseName, int frameId){
        std::string relativeImag = GetTestNode<std::string>(TESTCASE, caseName, "imag");
        std::string imagType = GetTestNode<std::string>(TESTCASE, caseName, "imagType");
        
        if (frameId == -1){
            return rootImgDir + relativeImag;
        }else{
            int fileNameLen = GetTestNode<int>(TESTCASE, caseName, "fileNameLen");
            
            std::stringstream ss;
            if (fileNameLen == -1){
                ss<<frameId <<"."<<imagType;
            }else{
                ss<<std::setw (fileNameLen)<< std::setfill ('0')<<frameId <<"."<<imagType;
            }
            std::string fileName1=ss.str();
            return rootImgDir + relativeImag + fileName1;
        }
    }
    
    int TestCaseParser::GetStartFrame(std::string caseName){
        return GetTestNode<int>(TESTCASE, caseName, "startFrame");
    }
    
    int TestCaseParser::GetEndFrame(std::string caseName){
        return GetTestNode<int>(TESTCASE, caseName, "endFrame");
    }
    
    std::string TestCaseParser::GetConfigDir(std::string caseName){
        std::string str = GetTestNode<std::string>(TESTCASE, caseName, "configName");
        return str;
    }
    
    template int
    TestCaseParser::GetTestNode(std::string rootName, std::string caseName, std::string param);
    template std::string
    TestCaseParser::GetTestNode(std::string rootName, std::string caseName, std::string param);
    
    template<typename T>
    T TestCaseParser::GetTestNode(std::string rootName, std::string caseName, std::string param){
        T re;
        cv::FileStorage fs;
        fs.open(xmlDir.c_str(), CV_STORAGE_READ);
        if(!fs.isOpened()) {
            std::cout<<"read test case config failed!!";
            return re;
        }
        cv::FileNode caseRootNode = fs[rootName];
        cv::FileNode caseNode = caseRootNode[caseName];
        if(caseNode.empty()){
            fs.release();
            std::cout<<"no such test case!!";
            return re;
        }
        cv::FileNode paramNode = caseNode[param];
        if (paramNode.empty()){
            return re;
        }
        re = T(paramNode);
        fs.release();
        return re;
    }
    
    std::vector<std::string> TestCaseParser::GetCaseNames(){
        std::vector<std::string> nameList;
        cv::FileStorage fs;
        fs.open(xmlDir.c_str() , CV_STORAGE_READ);
        if(!fs.isOpened()) {
            std::cout<<"read test case config failed!!";
            return nameList;
        }
        std::string root_node_name = "TestCase";
        cv::FileNode nodes_set = fs[root_node_name.c_str()];
        for(cv::FileNodeIterator iter = nodes_set.begin(); iter != nodes_set.end(); iter++) {
            nameList.push_back((*iter).name());
        }
        fs.release();
        return nameList;
    }
    
    void TestCaseParser::GetCamConfig(std::string caseName, float& fx, float& fy, float& cx, float& cy){
        std::string camName = GetTestNode<std::string>(TESTCASE, caseName, "Cam");
        if (camName ==""){
            std::cout <<"cannot find cam in test case config!!"<<std::endl;
            return;
        }
        fx = GetTestNode<float>(CAMERA, camName, "fx");
        fy = GetTestNode<float>(CAMERA, camName, "fy");
        cx = GetTestNode<float>(CAMERA, camName, "cx");
        cy = GetTestNode<float>(CAMERA, camName, "cy");
    }
}