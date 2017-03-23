//
//  paramParser.cpp
//  Project
//
//  Created by zhaiq on 8/10/16.
//
//

#include "paramParser.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <cassert>
#include <sstream>

namespace ygomi
{
    ParamParser::ParamParser()
    {
        m_params.clear();
    }
    
    ParamParser::~ParamParser()
    {
        m_params.clear();
    }
    
    template<typename T>
    void readSingleParam(const cv::FileStorage& fs,
                         const cv::FileNode& node,
                         const std::string& node_name,
                         bool bReadFromNode,
                         std::ostringstream& istr)
    {
        T ival;
        if(bReadFromNode) {
            node[node_name] >> ival;
        }
        else {
            fs[node_name] >> ival;
        }
        
        istr << ival;
    }

    void ParamParser::readParams(const cv::FileStorage& fs,
                                 const cv::FileNode& node,
                                 const cv::FileNodeIterator& sub_node)
    {
        static std::ostringstream istr;
        istr.str("");

        bool bExistSubNode           = sub_node != cv::FileNodeIterator();

        cv::FileNode working_node    = bExistSubNode ? (*sub_node) : node;
        const std::string& node_name = working_node.name();
        int type                     = working_node.type();
        
        switch (type) {
            case cv::FileNode::MAP: {
               for(cv::FileNodeIterator iter = working_node.begin(); iter != working_node.end(); iter++) {
                    readParams(fs, working_node, iter);
                }
                
                break;
            }
            case cv::FileNode::INT: {
                readSingleParam<int>(fs, node, node_name, bExistSubNode, istr);
                CV_Assert(!m_params.count(node_name));
                m_params[node_name] = istr.str();

                break;
            }
            case cv::FileNode::REAL: {
                readSingleParam<float>(fs, node, node_name, bExistSubNode, istr);
                CV_Assert(!m_params.count(node_name));
                m_params[node_name] = istr.str();

                break;
            }
            case cv::FileNode::STRING: {
                readSingleParam<std::string>(fs, node, node_name, bExistSubNode, istr);
                CV_Assert(!m_params.count(node_name));
                m_params[node_name] = istr.str();

                break;
            }
            default:
                throw "invalid configuration format.\n";
                break;
        }
    }
    
    bool ParamParser::init(const char* config)
    {
        CV_Assert(config);
        
        try {
            cv::FileStorage fs;
            fs.open(config, CV_STORAGE_READ);
            if(!fs.isOpened()) {
                throw "configuration file path is not correct!";
            }

            const std::string& node_name = "ParamNames";
            cv::FileNode nodes_set = fs[node_name.c_str()];
            for(cv::FileNodeIterator iter = nodes_set.begin(); iter != nodes_set.end(); iter++) {
                std::string node_name;
                *iter >> node_name;
                
                cv::FileNode node = fs[node_name];
                readParams(fs, node);
            }
            
            fs.release();
        } catch (...) {
            std::cout << "Caught an exception while parsing configuration file\n";
			return false;
        }
        
        return true;
    }
    
    void ParamParser::save(const char* path)
    {
        //todo:
    }
    
    bool ParamParser::checkParam(const char *name)
    {
        return m_params.count(name);
    }
    
    ////////////////////////////////////////////////////////////////////////////
    //
    //                              Read Handle
    //
    ////////////////////////////////////////////////////////////////////////////
    const std::string& ParamParser::parseString(const char* name)
    {
        CV_Assert(checkParam(name));
        return m_params.find(name)->second;
    }
    
    int ParamParser::parseInteger(const char* name)
    {
        const std::string& str = parseString(name);
        return atoi(str.c_str());
    }
    
    float ParamParser::parseFloat(const char* name)
    {
        const std::string& str = parseString(name);
        return atof(str.c_str());
    }
    
    std::vector<std::string> ParamParser::parseStringSet(const char* name)
    {
        CV_Assert(checkParam(name));
        const std::string& value = m_params.find(name)->second;
        std::istringstream istr;
        istr.str(value);
        
        std::vector<std::string> res;
        std::string str;
        while(istr >> str) {
            res.push_back(str);
        }
        
        return res;
    }
    
    std::vector<int> ParamParser::parseIntegerSet(const char* name)
    {
        std::vector<std::string> tempSet = parseStringSet(name);
        std::vector<int> res;
        res.reserve(tempSet.size());
        for(const auto& it : tempSet) {
            res.push_back(atoi(it.c_str()));
        }
        
        return res;
    }
    
    std::vector<float> ParamParser::parseFloatSet(const char* name)
    {
        std::vector<std::string> tempSet = parseStringSet(name);
        std::vector<float> res;
        res.reserve(tempSet.size());
        for(const auto& it : tempSet) {
            res.push_back(atof(it.c_str()));
        }
        
        return res;
    }
    
    
    ////////////////////////////////////////////////////////////////////////////
    //
    //                              Write Handle
    //
    ////////////////////////////////////////////////////////////////////////////
    void ParamParser::addInteger(const char* name, int v)
    {
        const int size = 64;
        char ch[size];
        memset(ch, ' ', sizeof(char)*size);
        sprintf(ch, "%d", v);
        m_params[name] = static_cast<std::string>(ch);
    }

    void ParamParser::addFloat(const char* name, float v)
    {
        const int size = 64;
        char ch[size];
        memset(ch, ' ', sizeof(char)*size);
        sprintf(ch, "%f", v);
        m_params[name] = static_cast<std::string>(ch);
    }
    
    void ParamParser::addString(const char* name, const std::string& v)
    {
        m_params[name] = v;
    }
    
    void ParamParser::addIntegerSet(const char* name, const std::vector<int>& v)
    {
        const int size = 64;
        char ch[size];
        std::string str;
        for(auto& it : v) {
            memset(ch, ' ', sizeof(char)*size);
            sprintf(ch, "%d", it);
            
            str += static_cast<std::string>(ch) + " ";
        }
        
        m_params[name] = str;
    }
    
    void ParamParser::addFloatSet(const char* name, const std::vector<float>& v)
    {
        const int size = 64;
        char ch[size];
        std::string str;
        for(auto& it : v) {
            memset(ch, ' ', sizeof(char)*size);
            sprintf(ch, "%f", it);
            str += static_cast<std::string>(ch) + " ";
        }
        
        m_params[name] = str;
    }
    
    void ParamParser::addStringSet(const char* name, const std::vector<std::string>& v)
    {
        std::string str;
        for(auto& it : v) {
            str += it + " ";
        }
        
        m_params[name] = str;
    }
}
