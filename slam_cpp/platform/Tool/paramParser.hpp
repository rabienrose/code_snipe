
#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <opencv2/core.hpp>

namespace ygomi
{
    class ParamParser
    {
    public:
        ParamParser();
        ~ParamParser();

    public:
        /**
         *
         */
        bool init(const char* config);
        
        //I/O
        /**
         *
         */
        void save(const char* path);
        
        //read handle
        /**
         *
         */
        const std::string& parseString(const char* name);
        
        /**
         *
         */
        int parseInteger(const char* name);
        
        /**
         *
         */
        float parseFloat(const char* name);
        
        /**
         *
         */
        std::vector<std::string> parseStringSet(const char* name);
        
        /**
         *
         */
        std::vector<int> parseIntegerSet(const char* name);
        
        /**
         *
         */
        std::vector<float> parseFloatSet(const char* name);
       
        //write handle.
        /**
         *
         */
        void addInteger(const char* name, int v);
        
        /**
         *
         */
        void addFloat(const char* name, float v);
        
        /**
         *
         */
        void addString(const char* name, const std::string& v);
        
        /**
         *
         */
        void addIntegerSet(const char* name, const std::vector<int>& v);
        
        /**
         *
         */
        void addFloatSet(const char* name, const std::vector<float>& v);
        
        /**
         *
         */
        void addStringSet(const char* name, const std::vector<std::string>& v);

        
        bool checkParam(const char* name);
        
    protected:
        void readParams(const cv::FileStorage& fs,
                       const cv::FileNode& node,
                       const cv::FileNodeIterator& sub_node = cv::FileNodeIterator());
        
    protected:
        //
        std::unordered_map<std::string, std::string> m_params;
    };
}