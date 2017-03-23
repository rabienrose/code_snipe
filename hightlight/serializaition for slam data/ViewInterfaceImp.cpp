#include "ViewInterfaceImp.h"
#include <iostream>
#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/noncopyable.hpp>
#include <boost/format.hpp>


namespace chamo {
    
    //save binary files
    class FBufferBin: public boost::noncopyable{
    public:
        enum _WRITE {write};
        enum _READ {read};
        
    public:
        FBufferBin(const std::string& name, _WRITE){
            fs_ = new std::fstream(name, std::ios::out);
            poa_ = new boost::archive::binary_oarchive(*fs_);
            pia_ = nullptr;
        }
        FBufferBin(const std::string& name, _READ){
            fs_ = new std::fstream(name, std::ios::in);
            poa_ = nullptr;
            pia_ = nullptr;
            if(fs_->good()){
                pia_ = new boost::archive::binary_iarchive(*fs_);
            }
        }
        
        ~FBufferBin(){
            if(poa_) delete poa_;
            if(pia_) delete pia_;
            if(fs_) delete fs_;
        }
        
        bool good(){
            if(fs_!=nullptr){
                return fs_->good();
            }
            return false;
        }
        
        boost::archive::binary_oarchive& w(){ return *poa_;}
        boost::archive::binary_iarchive& r(){ return *pia_;}
        
        template<typename _T>
        FBufferBin& operator<<(const _T& t){
            w()<<t;
            return *this;
        }
        template<typename _T>
        FBufferBin& operator>>(_T& t){
            r()>>t;
            return *this;
        }
        
        
    private:
        std::fstream* fs_;
        boost::archive::binary_oarchive *poa_;
        boost::archive::binary_iarchive *pia_;
    };

    
    void ViewInterfaceImp::ClearData(int type, int channel){
        if(type==1){
            viewData.MPLists[channel].clear();
        }
        if(type==2){
            viewData.KFLists[channel].clear();
        }
        if(type==3){
            viewData.RSLists[channel].clear();
        }
        if(type==4){
            viewData.FPLists[channel].clear();
        }
        if(type==5){
            viewData.IMLists[channel].clear();
        }
        if(type==6){
            viewData.TSLists[channel].clear();
        }
    }
    
    void ViewInterfaceImp::ClearAllData(){
        viewData.MPLists.clear();
        viewData.KFLists.clear();
        viewData.RSLists.clear();
        viewData.FPLists.clear();
        viewData.IMLists.clear();
        viewData.TSLists.clear();
        viewData.MPLists.resize(MAX_CHANNEL);
        viewData.KFLists.resize(MAX_CHANNEL);
        viewData.RSLists.resize(MAX_CHANNEL);
        viewData.FPLists.resize(MAX_CHANNEL);
        viewData.IMLists.resize(MAX_CHANNEL);
        viewData.TSLists.resize(MAX_CHANNEL);
    }
    
    int ViewInterfaceImp::SaveData(int type, int channel, std::string fileName){
        if(channel >=MAX_CHANNEL){
            return -1;
        }
        FBufferBin bf(fileName, FBufferBin::write);
        if(!bf.good())
            return -1;
        
        if(type==1){
            bf.w() & viewData.MPLists[channel];
        }
        if(type==2){
            bf.w() & viewData.KFLists[channel];
        }
        if(type==3){
            bf.w() & viewData.RSLists[channel];
        }
        if(type==4){
            bf.w() & viewData.FPLists[channel];
        }
        if(type==5){
            bf.w() & viewData.IMLists[channel];
        }
        if(type==6){
            bf.w() & viewData.TSLists[channel];
        }
        return 1;
    }
    
    int ViewInterfaceImp::SaveAll(std::string fileName){
        if(fileName ==""){
            if(autoSaveAddr == " "){
                return -1;
            }
            fileName = autoSaveAddr;
        }
        FBufferBin bf(fileName, FBufferBin::write);
        if(!bf.good())
            return -1;
        
        bf.w() & viewData;
        saveCount++;
        //std::cout<<"save times: "<<saveCount<<std::endl;
        return 1;
    }
    
    int ViewInterfaceImp::ReadListFile(std::string fileName, int type, int channel){
        int count=0;
        if(channel >=MAX_CHANNEL){
            return -1;
        }
        FBufferBin bf(fileName, FBufferBin::read);
        if(!bf.good())
            return -1;
        if(type==1){
            std::vector<MapPointCpp> item;
            bf.r() & item;
            viewData.MPLists[channel] =item;
            count =viewData.MPLists[channel].size();
        }
        if(type==2){
            std::vector<KeyFrameCpp> item;
            bf.r() & item;
            viewData.KFLists[channel] =item;
            count =viewData.KFLists[channel].size();
        }
        if(type==3){
            std::vector<RoadSegCpp> item;
            bf.r() & item;
            viewData.RSLists[channel] =item;
            count =viewData.RSLists[channel].size();
        }
        if(type==4){
            std::vector<FeaturesCpp> item;
            bf.r() & item;
            viewData.FPLists[channel] =item;
            count =viewData.FPLists[channel].size();
        }
        if(type==5){
            std::vector<ImageCpp> item;
            bf.r() & item;
            viewData.IMLists[channel] =item;
            count =viewData.IMLists[channel].size();
        }
        if(type==6){
            std::vector<TSCpp> item;
            bf.r() & item;
            viewData.TSLists[channel] =item;
            count =viewData.TSLists[channel].size();
        }
        return count;
    }
    
    int ViewInterfaceImp::ReadViewDataFile(std::string fileName){
        FBufferBin bf(fileName, FBufferBin::read);
        if(!bf.good())
            return -1;
        
        bf.r() & viewData;
        return 1;
    }
    
    void ViewInterfaceImp::AddMapPoint(int channel, MapPointCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.MPLists[channel].push_back(*data);
    
    }
    
    void ViewInterfaceImp::AddKeyFrame(int channel, KeyFrameCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.KFLists[channel].push_back(*data);
    }
    
    void ViewInterfaceImp::AddRoadSeg(int channel, RoadSegCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.RSLists[channel].push_back(* data);
    }
    
    void ViewInterfaceImp::AddFeaturePoint(int channel, FeaturesCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.FPLists[channel].push_back(* data);
    }
    
    void ViewInterfaceImp::AddImage(int channel, ImageCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.IMLists[channel].push_back(* data);
    }
    
    void ViewInterfaceImp::AddTraficSign(int channel, TSCpp* data){
        if(channel >= MAX_CHANNEL){
            return;
        }
        viewData.TSLists[channel].push_back(*data);
    
    };
}
