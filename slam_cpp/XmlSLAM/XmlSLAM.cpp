#include "XmlSLAM.hpp"
#include <string>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace chamo
{
    void XmlSLAM::SaveData(std::string fileName){
        cv::FileStorage fs;
        fs.open(fileName.c_str(), CV_STORAGE_WRITE);
        if(!fs.isOpened()) {
            std::cout<<"read file failed!!";
            return;
        }
        write(fs);
        fs.release();
    }
    void XmlSLAM::ReadData(std::string fileName){
        cv::FileStorage fs;
        fs.open(fileName.c_str(), CV_STORAGE_READ);
        if(!fs.isOpened()) {
            std::cout<<"read file failed!!";
            return;
        }
        read(fs.root());
        fs.release();
    }
    
    void XmlSLAM::ClearData(){
        mapoints.clear();
        frames.clear();
    }
    
    void XmlSLAM::write(cv::FileStorage& fs) const
    {
        fs<<"frames"<<"[";
        for (std::map<int, Frame>::const_iterator it=frames.begin();it!=frames.end();it++){
            fs<<it->second;
        }
        fs<<"]";
        fs<<"mapoints"<<"[";
        for (std::unordered_map<int, MapPoint>::const_iterator it=mapoints.begin();it!=mapoints.end();it++){
            fs<<it->second;
        }
        fs<<"]";
    }
    
    void XmlSLAM::read(const cv::FileNode& node)
    {
        cv::FileNode n;
        cv::FileNodeIterator it;
        cv::FileNodeIterator it_end;
        n = node["frames"];
        if (n.type() != cv::FileNode::SEQ)
        {
            return;
        }
        
        it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it){
            Frame frame;
            *it>>frame;
            frames[frame.id] = frame;
        }
        
        n = node["mapoints"];
        if (n.type() != cv::FileNode::SEQ)
        {
            return;
        }
        
        it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it){
            MapPoint mappoint;
            *it>>mappoint;
            mapoints[mappoint.id] = mappoint;
        }
    }
    
    void write(cv::FileStorage& fs, const std::string&, const XmlSLAM& x)
    {
        x.write(fs);
    }
    
    void read(const cv::FileNode& node, XmlSLAM& x, const XmlSLAM& default_value = XmlSLAM())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }
    
    void Frame::write(cv::FileStorage& fs) const
    {
        fs<<"{"<<"id"<<id<<"name"<<name<<"pose"<<pose<<"type"<<type;
        fs<<"kps"<<"[";
        for (int i=0;i<kps.size();i++){
            fs<<kps[i];
        }
        fs<<"]";
        fs<<"}";
    }
    
    void Frame::read(const cv::FileNode& node)
    {
        name = (std::string)node["name"];
        type = (int)node["type"];
        id = (int)node["id"];
        node["pose"]>>pose;
        cv::FileNode n = node["kps"];
        if (n.type() != cv::FileNode::SEQ)
        {
            return;
        }
        
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it){
            KeyPoint kp;
            *it>>kp;
            kps.push_back(kp);
        }
    }
    
    void write(cv::FileStorage& fs, const std::string&, const Frame& x)
    {
        x.write(fs);
    }
    
    void read(const cv::FileNode& node, Frame& x, const Frame& default_value = Frame())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }
    
    void KeyPoint::write(cv::FileStorage& fs) const
    {
        fs<<"{"<<"uv"<<uv<<"octave"<<octave;
        fs<<"MPIDs"<<"[";
        for (int i=0;i<MPIDs.size();i++){
            fs<<MPIDs[i];
        }
        fs<<"]";
        fs<<"}";
    }
    
    void KeyPoint::read(const cv::FileNode& node)
    {
        node["uv"]>>uv;
        octave = (int)node["octave"];
        cv::FileNode n = node["MPIDs"];
        if (n.type() != cv::FileNode::SEQ)
        {
            std::cerr << "MPIDs is not a sequence! FAIL" << std::endl;
        }
        
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it){
            int id;
            id = (int)*it;
            MPIDs.push_back(id);
        }
    }

    void write(cv::FileStorage& fs, const std::string&, const KeyPoint& x)
    {
        x.write(fs);
    }
    
    void read(const cv::FileNode& node, KeyPoint& x, const KeyPoint& default_value = KeyPoint())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }
    
    void MapPoint::write(cv::FileStorage& fs) const
    {
        fs<<"{"<<"id"<<id<<"isBad"<<isBad<<"posi"<<posi;
        fs<<"tracks"<<"[";
        for (int i=0;i<tracks.size();i++){
            fs<<"{";
            fs<<"frameId"<<tracks[i].first;
            fs<<"kpId"<<tracks[i].second;
            fs<<"}";
        }
        fs<<"]";
        fs<<"}";

    }
    
    void MapPoint::read(const cv::FileNode& node)
    {
        node["id"]>>id;
        node["isBad"]>>isBad;
        node["posi"]>>posi;
        cv::FileNode n = node["tracks"];
        if (n.type() != cv::FileNode::SEQ)
        {
            std::cerr << "tracks is not a sequence! FAIL" << std::endl;
        }
        
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it){
            int frameId = (int)(*it)["frameId"];
            int kpId = (int)(*it)["kpId"];
            tracks.push_back(std::pair<int,int>(frameId, kpId));
        }
    }
    
    void write(cv::FileStorage& fs, const std::string&, const MapPoint& x)
    {
        x.write(fs);
    }
    
    void read(const cv::FileNode& node, MapPoint& x, const MapPoint& default_value = MapPoint())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }
}
