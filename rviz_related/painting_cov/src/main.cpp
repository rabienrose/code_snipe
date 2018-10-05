#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <show_map/algo.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "painting_cov");
    ros::NodeHandle n;
    ros::ServiceClient client;
    std::cout<<"start converting"<<std::endl;
    client = n.serviceClient<show_map::algo>("algo_service");
    std::ifstream paintFile("../../../data/paints_nissan_958.kml");
    bool firstRef= false;
    cv::Point3d refref_posi;
    std::vector<std::vector<cv::Point3f> > relPosiList;
    if (paintFile.is_open()){
        std::cout<<"open file succ"<<std::endl;
        int line_count=0;
        if(1)
        {
            while(ros::ok()){
                char newseq_c[1024];
                paintFile.getline(newseq_c, 1024);
    //             std::cout<<newseq_c<<std::endl;
                if(newseq_c[0]==0){
                    break;
                }
                if((newseq_c[5]=='>' && newseq_c[6]=='l' && newseq_c[7]=='i' && newseq_c[8]=='n' && newseq_c[9]=='e')||
                (newseq_c[5]=='>' && newseq_c[6]=='a' && newseq_c[7]=='r' && newseq_c[8]=='i' && newseq_c[9]=='t'&& newseq_c[10]=='i' && newseq_c[11]=='f' && newseq_c[12]=='i' && newseq_c[13]=='c')){
                    char long_str_c[10240];
                    paintFile.getline(long_str_c, 10240);
                    std::vector<cv::Point3f> newPosiList;
    //                 int number = 0,frequence = 10;
                    while(ros::ok()){
                        char newposi_c[1024];
                        paintFile.getline(newposi_c, 1024);
                        if (newposi_c[0]=='<'){
                            break;
                        }
                        //std::cout<<newposi_c<<std::endl;
                        cv::Point3d ref_posi;
                        cv::Point3f rel_posi;
                        sscanf(newposi_c, "%lf,%lf,%lf",&ref_posi.x, &ref_posi.y, &ref_posi.z);
    //                     number ++;
                        
    //                     if(!firstRef){
    //                         refref_posi = ref_posi;
    //                         firstRef=true;
    //                         ref_posi.x=0;
    //                         ref_posi.y=0;
    //                         ref_posi.z=0;
    //                     }else
    //                     {
    //                     if(frequence == number)
    //                     {
    //                         number = 0;
                            show_map::algo algo_srv;
                            algo_srv.request.absGPS.x=-83.12255859;
                            algo_srv.request.absGPS.y=42.49511719;
                            algo_srv.request.absGPS.z=0;
                            algo_srv.request.refGPS.x=ref_posi.x;
                            algo_srv.request.refGPS.y=ref_posi.y;
                            algo_srv.request.refGPS.z=ref_posi.z + 1.5;//
                            client.call(algo_srv);
                            rel_posi.x=algo_srv.response.relGPS.x;
                            rel_posi.z=algo_srv.response.relGPS.y;
                            rel_posi.y=algo_srv.response.relGPS.z;
    //                         std::cout<<"abs_posi: "<<ref_posi<<std::endl;
    //                         std::cout<<"ref_posi: "<<refref_posi<<std::endl;
    //                         std::cout<<"rel_posi: "<<ref_posi<<std::endl;
    //                     }
                            newPosiList.push_back(rel_posi);
    //                     }
                    }
                    line_count++;
                    std::cout<<"line count: "<<line_count<<std::endl;
                    relPosiList.push_back(newPosiList);
                }
            }
        }
        if(0)
        {
            while(ros::ok())
            {
                char newseq_c[1024];
                paintFile.getline(newseq_c, 1024);  
//                 std::cout<<newseq_c<<std::endl;
                if(newseq_c[0]==0)
                {
                    break;
                }
                if((newseq_c[6]=='<' && newseq_c[7]=='L' && newseq_c[8]=='i' && newseq_c[9]=='n' && newseq_c[10]=='e'))
                {
                    std::cout<<"data line"<<std::endl;
                    char long_str_c[1024000]; 
                    std::vector<cv::Point3f> newPosiList;
                    float lastAltitude = 0;
                    while(ros::ok())
                    {
                        paintFile.getline(long_str_c, 1024000);
                        if (long_str_c[0]=='<')
                        {
                            break;
                        }
                        const char *d = " ";
                        char *out;
                        out = strtok(long_str_c,d);
                        cv::Point3d ref_posi;
                        cv::Point3f rel_posi;
                        while(out)
                        {
                            sscanf(out, "%lf,%lf,%lf",&ref_posi.x, &ref_posi.y, &ref_posi.z);
                            if(0 == ref_posi.z)
                            {
                                ref_posi.z = lastAltitude;
                            }
                            lastAltitude =  ref_posi.z;
                            if(0 == lastAltitude)
                            {
                                std::cout<<"error :lastAltitude = 0"<<std::endl;
                            }
                            std::cout<<"ref_posi:"<<ref_posi<<std::endl;
                            out=strtok(NULL,d);
                            show_map::algo algo_srv;
                            
//                             [lon=-83.14453125],[lat=42.47314453],[alt=0]   710 case
//                             [lon=-83.03466797],[lat=42.48413086],[alt=0]   015 case
//                             [lon=-83.12255859],[lat=42.49511719],[alt=0]   602 case
//                             [lon=-83.12255859],[lat=42.49511719],[alt=0]   958 case
//                             [lon=-83.12255859],[lat=42.49511719],[alt=0]  2017-11-13_T_16-36-28.441_GMT_keyFrame_poses.txt
//                             [lon=-83.12255859],[lat=42.49511719],[alt=0]  2017-11-13_T_16-26-36.472_GMT_keyFrame_poses.txt
                            
                            algo_srv.request.absGPS.x=-83.12255859;
                            algo_srv.request.absGPS.y=42.49511719;
                            algo_srv.request.absGPS.z=0;
                            algo_srv.request.refGPS.x=ref_posi.x;
                            algo_srv.request.refGPS.y=ref_posi.y;
                            algo_srv.request.refGPS.z=ref_posi.z + 1.5;//
                            client.call(algo_srv);
                            rel_posi.x=algo_srv.response.relGPS.x;
                            rel_posi.z=algo_srv.response.relGPS.y;
                            rel_posi.y=algo_srv.response.relGPS.z;
                            newPosiList.push_back(rel_posi);
                        }
                    }
                    line_count++;
                    std::cout<<"line count: "<<line_count<<std::endl;
                    relPosiList.push_back(newPosiList);
                }
            }   
        }
        
        std::stringstream ss;
        
        ss<<"newseq"<<std::endl;
        ss<<"0000000"<<std::endl;
        ss<<refref_posi<<std::endl;
        
        for (int i=0; i<relPosiList.size();i++){
            if (relPosiList[i].size()>0){
                ss<<"newline"<<std::endl;
                ss<<"solid"<<std::endl;
                for (int j=0;j<relPosiList[i].size();j++){
                    ss<<relPosiList[i][j].x<<" "<<relPosiList[i][j].y<<" "<<relPosiList[i][j].z<<std::endl;
                }
            }
        }
        std::ofstream out("../../../data/paints_nissan_958.txt");
        out<<ss.str();
        out.close();
    }
    return 0;
}
