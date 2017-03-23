#define NO_ALGO_LIB

#include "ViewInterfaceImp.h"
#include <boost/filesystem.hpp>

#ifndef NO_ALGO_LIB
#include "vehicleSystem.h"
#include "AlgoInterfaceImp.h"
#endif

#include <string>
#include <vector>
#include <iostream>
#include <sstream>





#define MAX_PARAM 8


std::string getTime(){
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
    std::string str(buffer);
    return str;
}

#ifndef NO_ALGO_LIB

NO_ALGO_LIBalgo::vehicle::VehicleSystem* vehicle = NULL;
#endif
algo::ViewInterfaceImp* pViewI=NULL;
int curAlgo=-1;
std::vector<std::string> filenames;
std::vector<std::string> params;
int max_steps = -1;

extern "C" {
    void SetParam(int ind, char* valc){
        std::string val(valc);
        std::cout<<"SetParam: "<<val<<std::endl;
        if(ind<MAX_PARAM){
            params[ind] = val;
        }
    }

    // 1: SLAM, 2: GPE, 3: Server
    int InitAlgo(int algoType){
#ifndef NO_ALGO_LIB

        curAlgo =algoType;
        if(algoType == 3){
        }
        if(algoType == 2){
        }
        if(algoType == 1){
            vehicle= new algo::vehicle::VehicleSystem();
            pViewI= new algo::ViewInterfaceImp();
            pViewI->autoSaveAddr =params[3];
            
            algo::AlgoCamParam   *pCamParam = new algo::AlgoCamParam(params[2]);
            algo::AlgoSlamConfig *pSlamCfg  = new algo::AlgoSlamConfig(params[2]);
            algo::AlgoImage      *pImage    = new algo::AlgoImage(params[0]);
            max_steps =pImage->getNum();
            std::cout<<"max_steps: "<<max_steps<<std::endl;
            if(!vehicle->initialize(pCamParam, pSlamCfg, pImage, NULL, NULL, NULL,
                                    params[1], pViewI, false))
            {
                std::cout<<"vehicle ini failed"<<std::endl;
                return -1;
            }
            std::cout<<"init succuss!!"<<std::endl;
            vehicle->trackAndLocalization();

        }
#endif
        return max_steps;
    }

    int ReadSingleData(char* fileNameC, int type, int channel){
        std::string fileName(fileNameC);
        return pViewI->ReadListFile(fileName, type, channel);
    }

    int ReadAllData(char* fileNameC){
        std::string fileName(fileNameC);
        return pViewI->ReadViewDataFile(fileName);
    }

    int SaveList(int type, int channel, char* fileNameC){
        std::string fileName(fileNameC);
        return pViewI->SaveData(type, channel, fileName);
    }

    int SaveResult(char* fileNameC){
        std::string fileName(fileNameC);
        return pViewI->SaveAll(fileName);
    }

    int GetListCount(int type, int channel){
        int count=-1;
        if(channel< MAX_CHANNEL){
            if(type == 1){
                count =pViewI->viewData.MPLists[channel].size();
            }
            if(type == 2){
                count =pViewI->viewData.KFLists[channel].size();
            }
            if(type == 3){
                count =pViewI->viewData.RSLists[channel].size();
            }
            if(type == 4){
                count =pViewI->viewData.FPLists[channel].size();
            }
            if(type == 5){
                count =pViewI->viewData.IMLists[channel].size();
            }
            if(type == 6){
                count =pViewI->viewData.TSLists[channel].size();
            }
        }
        return count;
    }

    algo::KeyFrameCppC2Sharp GetKFListCpp(int channel, int ind, int** pkfLinks1, int** pkfLinks2, int** pkfLinks3, int** pkfLinksVal1, int** pkfLinksVal2, int** pkfLinksVal3, int** pmpLinks){
        algo::KeyFrameCpp item;
        algo::KeyFrameCppC2Sharp itemCSharp;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.KFLists[channel].size()){
                item = pViewI->viewData.KFLists[channel][ind];
                itemCSharp.id = item.id;
                itemCSharp.x = item.x;
                itemCSharp.y = item.y;
                itemCSharp.z = item.z;
                itemCSharp.r00 = item.r00;
                itemCSharp.r01 = item.r01;
                itemCSharp.r02 = item.r02;
                itemCSharp.r10 = item.r10;
                itemCSharp.r11 = item.r11;
                itemCSharp.r12 = item.r12;
                itemCSharp.r20 = item.r20;
                itemCSharp.r21 = item.r21;
                itemCSharp.r22 = item.r22;
                itemCSharp.r = item.r;
                itemCSharp.g = item.g;
                itemCSharp.b = item.b;
                itemCSharp.filter1 = item.filter1;
                itemCSharp.filter2 = item.filter2;
                itemCSharp.filter3 = item.filter3;
                itemCSharp.kfLinksCount1 = item.kfLinks1.size();
                itemCSharp.kfLinksCount2 = item.kfLinks2.size();
                itemCSharp.kfLinksCount3 = item.kfLinks3.size();
                itemCSharp.mpLinksCount = item.mpLinks.size();
                *pkfLinks1 =  &item.kfLinks1[0];
                *pkfLinks2 =  &item.kfLinks2[0];
                *pkfLinks3 =  &item.kfLinks3[0];
                *pkfLinksVal1 =  &item.kfLinksVal1[0];
                *pkfLinksVal2 =  &item.kfLinksVal2[0];
                *pkfLinksVal3 =  &item.kfLinksVal3[0];
                *pmpLinks =  &item.mpLinks[0];
            }
        }
        return itemCSharp;
    }

    algo::MapPointCpp GetMPListCpp(int channel, int ind){
        algo::MapPointCpp item;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.MPLists[channel].size()){
                item = pViewI->viewData.MPLists[channel][ind];
            }
        }
        return item;
    }

    algo::RoadSegCpp GetRSListCpp(int channel, int ind){
        algo::RoadSegCpp item;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.RSLists[channel].size()){
                item = pViewI->viewData.RSLists[channel][ind];
            }
        }
        return item;
    }

    algo::FeaturesCpp GetFPListCpp(int channel, int ind){
        algo::FeaturesCpp item;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.FPLists[channel].size()){
                item = pViewI->viewData.FPLists[channel][ind];
            }
        }
        return item;
    }

    algo::ImageCppC2Sharp GetIMListCpp(int channel, int ind, unsigned char** pData){
        algo::ImageCppC2Sharp item;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.IMLists[channel].size()){
                item.id =pViewI->viewData.IMLists[channel][ind].id;
                item.w =pViewI->viewData.IMLists[channel][ind].w;
                item.h =pViewI->viewData.IMLists[channel][ind].h;
                //std::vector<unsigned char> ddd(item.w*item.h,200);
                //for (int i=0;i<item.w;i++){
                //    ddd[100*item.w+i] = 0;
                //}
                //pViewI->viewData.IMLists[channel][ind].data = ddd;
                *pData =  &pViewI->viewData.IMLists[channel][ind].data[0];
            }
        }
        return item;
    }
    
    algo::TSCpp GetTSListCpp(int channel, int ind){
        algo::TSCpp item;
        if(channel< MAX_CHANNEL){
            if(ind < pViewI->viewData.TSLists[channel].size()){
                item = pViewI->viewData.TSLists[channel][ind];
            }
        }
        return item;
    }

    void NewViewHandle(){
        freopen("debug.txt", "w", stdout);
        std::cout<<getTime()<<std::endl;
        pViewI = new algo::ViewInterfaceImp();
        params.resize(MAX_PARAM);
    }

}
