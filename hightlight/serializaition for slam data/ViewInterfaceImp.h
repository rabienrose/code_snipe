#ifndef SERVER_INTERFACE_H
#define SERVER_INTERFACE_H

#define MAX_CHANNEL 1024

#include <vector>
#include <string>

namespace chamo {
    struct KeyFrameCpp{
        KeyFrameCpp(){
            r=0;g=0;b=0;
        }
        int id=-1;
        //the position of the camera
        float x,y,z;
        //the oritation of the camera, p.s. these are not the translation and rotation of pose.
        //Basically, it is the translation and rotation of inverse matrix of pose
        //Define: Pose means the matrix used in the projection from 3d points to 2d pixels, it is the transform matrix from world coordination to camera coordination
        float r00,r01,r02;
        float r10,r11,r12;
        float r20,r21,r22;
        //color of the camera
        int r,g,b;
        //values assign to each camera, in the tool, there is function to highlight the camera with the value in the specific range. You can assign three independent value to each camera.
        int filter1=0;
        int filter2=0;
        int filter3=0;
        //links to other cameras, use the keyframeId to specify the target mp.
        std::vector<int> kfLinks1;
        std::vector<int> kfLinks2;
        std::vector<int> kfLinks3;
        //each link has a value come with it, you can only who the links with the value in the specific range.
        std::vector<int> kfLinksVal1;
        std::vector<int> kfLinksVal2;
        std::vector<int> kfLinksVal3;
        //links to mps. use the mapid to specify the target mp.
        std::vector<int> mpLinks;
        
        //serialization of boost, you don't need to take into account of this.
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &x &y &z &r00 &r01 &r02 &r10 &r11 &r12 &r20 &r21 &r22 &r &g &b &filter1 &filter2 &filter3 &kfLinks1 &kfLinks2 &kfLinks3 &kfLinksVal1 &kfLinksVal2 &kfLinksVal3 &mpLinks;
        }
    };
    
    struct MapPointCpp{
        int id=-1;
        // position of the mp
        float x,y,z;
        //color of the mp
        int r,g,b;
        //values assign to each mp, in the tool, there is function to highlight the mp with the value in the specific range. You can assign three independent value to each mp.
        int filter1=0;
        int filter2=0;
        int filter3=0;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &x &y &z &r &g &b &filter1 &filter2 &filter3;
        }
    };
    
    //deprecated now
    struct RoadSegCpp{
        int id=-1;
        float x,y,z;
        float nx, ny,nz;
        float halfWR, halfWL;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &x &y &z &nx &ny &nz;
        }
    };
    
    //deprecated now
    struct FeaturesCpp{
        int id=-1;
        int u;
        int v;
        int r;
        int lastU;
        int lastV;
        int red=0, green=0, blue=0;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &u &v &r &lastU &lastV &red &green &blue;
        }
    };
    
    //deprecated now
    struct ImageCpp{
        int id=-1;
        int h;
        int w;
        std::vector<unsigned char> data;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &h &w &data;
        }
    };
    
    //deprecated now
    struct TSCpp{
        int id=-1;
        float x,y,z;
        float forwardx,forwardy,forwardz;
        int typeId;
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &id &x &y &z &forwardx &forwardy &forwardz &typeId;
        }
    };
    
    struct ImageCppC2Sharp{
        int id;
        int h;
        int w;
    };
    
    struct KeyFrameCppC2Sharp{
        int id=-1;
        float x,y,z;
        float r00,r01,r02;
        float r10,r11,r12;
        float r20,r21,r22;
        int r,g,b;
        int filter1=0;
        int filter2=0;
        int filter3=0;
        int kfLinksCount1;
        int kfLinksCount2;
        int kfLinksCount3;
        int mpLinksCount;
    };
    
    struct ViewData{
        std::vector<std::vector<MapPointCpp> > MPLists;
        std::vector<std::vector<KeyFrameCpp> > KFLists;
        std::vector<std::vector<RoadSegCpp> > RSLists;
        std::vector<std::vector<FeaturesCpp> > FPLists;
        std::vector<std::vector<ImageCpp> > IMLists;
        std::vector<std::vector<TSCpp> > TSLists;
        std::vector<std::vector<std::string> > channelNames;
        ViewData(){
            MPLists.resize(MAX_CHANNEL);
            KFLists.resize(MAX_CHANNEL);
            RSLists.resize(MAX_CHANNEL);
            FPLists.resize(MAX_CHANNEL);
            IMLists.resize(MAX_CHANNEL);
            TSLists.resize(MAX_CHANNEL);
        }

        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &MPLists &KFLists &RSLists &FPLists &IMLists &TSLists;
        }
    };
    
    
    class ViewInterfaceImp{
    public:
        //append one mp to viewData in specific channel
        virtual void AddMapPoint(int channel, MapPointCpp* data);
        //append one kf to viewData in specific channel
        virtual void AddKeyFrame(int channel, KeyFrameCpp* data);
        //deprecated now
        virtual void AddRoadSeg(int channel, RoadSegCpp* data);
        //deprecated now
        virtual void AddFeaturePoint(int channel, FeaturesCpp* data);
        //deprecated now
        virtual void AddImage(int channel, ImageCpp* data);
        //deprecated now
        virtual void AddTraficSign(int channel, TSCpp* data);
        //clear specific type of data in specific channel
        virtual void ClearData(int type, int channel);
        virtual void ClearAllData();
        //save all specific type of data, so you need use ReadAList button to read the file in the visualization tool, if you use this way to save data
        virtual int SaveData(int type, int channel, std::string fileName);
        //save all types of data, so you need use ReadResult button to read the file in the visualization tool, if you use this way to save data
        virtual int SaveAll(std::string fileName);
        
        // just read all data, the channel that the data is stored in the file
        int ReadViewDataFile(std::string fileName);
        //read a specific type of data in the specific channel
        int ReadListFile(std::string fileName, int type, int channel);
        ViewData viewData;
        std::string autoSaveAddr=" ";
        int saveCount=0;
        
    };
}


#endif