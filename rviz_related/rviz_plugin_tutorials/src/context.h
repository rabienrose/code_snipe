#ifndef CONTEXT_H
#define CONTEXT_H
#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <unordered_map>
#include <vector>

class LocContext
{
    public:
        ~LocContext();
        static LocContext *getInstance();
        
        inline std::string dirName(const std::string& path) {                                                                                                                                                      
            static std::string dir = ".";
            std::size_t pos = path.rfind("/");
            if (pos != std::string::npos) {
                dir = path.substr(0, pos);
            }
            return dir;
        }

        std::vector<std::string> split(const std::string &s, char delim) {
            std::stringstream ss(s);
            std::string item;
            std::vector<std::string> tokens;
            while (getline(ss, item, delim)) {
                tokens.push_back(item);
            }
            return tokens;
        }

        std::string trim(const std::string &s, char comment_char = '#')
        {
            std::string result;
            std::string::size_type start_pos = s.find_first_not_of(" \t");
            if(start_pos == std::string::npos)
                return "";
            if(s.at(start_pos) == comment_char)
                return "";

            std::string::size_type end_pos = s.find_last_not_of(" \t#");
            if(end_pos == std::string::npos)
                return "";

           return s.substr(start_pos, end_pos - start_pos + 1);
        }



        inline std::string getInstallDir()
        {
            return installDir;
        }

        inline std::string getIconDir()
        {
            return iconDir;
        }

        inline std::string getPaintDir()
        {
            return paintDir;
        }

        inline std::string getParam(std::string type, std::string name)
        {
            if(settings.find(type) != settings.end()) {
                std::unordered_map<std::string, std::string>& setting = settings[type];
                if(setting.find(name) != setting.end()) {
                    //std::cout<<"get "<<type<<"."<<name<<"="<<setting[name]<<std::endl;
                    return setting[name];
                }
                else {
                    std::cerr << "getting illegal param name " << "\"" << name << "\"" << " of type: \"" << type << "\""
                              << std::endl;
                    return "";
                }
            }
            else {
                std::cerr << "getting illegal setting type " << "\"" << type << "\"" << std::endl;
                return "";
            }
        }

        inline void setParam(std::string type, std::string name, std::string value)
        {
            if(settings.find(type) != settings.end()) {
                std::unordered_map<std::string, std::string>& setting = settings[type];
                if(setting.find(name) != setting.end()) {
                    //std::cout<<"set "<<type<<"."<<name<<" from "<<setting[name] << " to " << value <<std::endl;
                    setting[name] = value;
                }
                else
                    std::cerr<<"setting non-exist param name "<< "\"" << name << "\"" << " of type: \"" << type << "\"" << std::endl;
            }
            else
                std::cerr<<"setting non-exist setting type " << "\"" << type << "\"" << std::endl;
        }

        void load(std::string settingFile);
        void save(std::string settingFile);

    private:
        //func
        LocContext(const LocContext&);
        LocContext& operator = (const LocContext&); 
        LocContext();

    private:
        //value
        std::string sourceDir;
        std::string installDir;
        std::string iconDir;
        std::string paintDir;
        std::string confDir;
        std::unordered_map<std::string, std::unordered_map<std::string, std::string> >settings;


};


#endif
