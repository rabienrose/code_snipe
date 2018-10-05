
#include "context.h"
#include <iostream>
#include <fstream>

LocContext::~LocContext()
{

}

LocContext *LocContext::getInstance()
{
   static LocContext instance;
   return &instance;
}

LocContext::LocContext()
{

    //installDir = dirName(dirName(exePath())); 

    char * env_value ;
    env_value = getenv("basepath");
    if(!env_value)
    {
        installDir = "/opt/ygomi/algorithm_localisation/bin/";
        sourceDir = "";
    }
    else
    {
        installDir = env_value;
        sourceDir = std::string(env_value) + "/src/";

    }
    installDir = dirName(installDir); 


    iconDir = installDir + "/share/rviz_plugin_tutorials/icon/";
    paintDir = installDir + "/share/show_map/paints_res/";

    confDir = installDir + "/share/rviz_plugin_tutorials/conf/";
    load(confDir + "settings.txt");

}

void LocContext::save(std::string settingFile) {}
void LocContext::load(std::string settingFile)
{
    // load setting
    std::ifstream ifs(settingFile.c_str(), std::ios::in);

    std::unordered_map<std::string, std::string> *pCurrentMap = 0;
    if(!ifs.is_open()) {
        if(!sourceDir.empty())
        {
            settingFile = sourceDir + "/rviz_plugin_tutorials/conf/settings.txt";
            iconDir = sourceDir + "/rviz_plugin_tutorials/icon/";
            paintDir = sourceDir + "/show_map/paints_res/";
            ifs.open(settingFile.c_str());
        }

        if(!ifs.is_open()) {
            std::cerr << "error opening setting file: " << settingFile << std::endl;
            exit(1);
        }
    }

    if(ifs.is_open())
    {
        std::cout<<"load setting: " << settingFile<<std::endl;
        char buffer[1024];
        int line_count = 0;
        while(!ifs.eof()) {
            ifs.getline(buffer, 1024);
            ++line_count;
            if(buffer[0] == '#' || buffer[0] == '\0')
                continue;

            // std::cout<<"line: "<< line_count << buffer << std::endl;
            std::vector<std::string> entries = split(trim(buffer), '=');
            if(entries.size() != 2) {
                if(entries.size() == 1) {
                    // std::cout<<"line: "<< line_count << " " << entries[0]<<std::endl;
                    std::string line = trim(entries[0]);
                    // std::cout<<"after trim: " << line <<std::endl;
                    if(line.empty())
                        continue;

                    if(line == "}") {
                        pCurrentMap = 0;
                        continue;
                    }

                    if(line.back() == '{') {
                        line.pop_back();
                        line = trim(line);
                        // std::cout<<"end with { "<< line <<std::endl;
                        settings[line] = std::unordered_map<std::string, std::string>();
                    }
                    else{
                        while(!ifs.eof()) {
                            ifs.getline(buffer, 1024);
                            ++line_count;
                            if(buffer[0] == '#' || buffer[0] == '\0')
                                continue;

                            // std::cout<<"line: "<< line_count << buffer <<std::endl;
                            // std::cout<<"after trim: " << trim(buffer) <<std::endl;
                            if(trim(buffer) == "{") {
                                settings[line] = std::unordered_map<std::string, std::string>();
                                std::cout<<"type: "<<line<<std::endl;
                                break;
                            }
                        }
                    }

                    pCurrentMap = &settings[line];
                }
                else {

                    // 0
                }
            }
            else
            {
                if(pCurrentMap) {
                    std::unordered_map<std::string, std::string>& setting = *pCurrentMap;

                    // std::cout<<"\""<<entries[0]<<"="<<entries[1]<<"\""<<std::endl;
                    setting[entries[0]] = entries[1];
                }
            }
        }
    }
}
