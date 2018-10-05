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
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <show_map/algo.h>
#include "../../rviz_plugin_tutorials/src/context.h"

inline std::string exePath()
{
    static std::string path;
    if (path.empty())
    {
        char buf[256] = {0};
        ssize_t ret = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (ret == -1)
        {
            path = "/opt/ygomi/algorithm_localisation/bin/run.sh";
        }
        else
        {
            path = buf;
        }
    }
    return path;
}

inline std::string dirName(const std::string &path)
{
    static std::string dir = ".";
    std::size_t pos = path.rfind("/");
    if (pos != std::string::npos)
    {
        dir = path.substr(0, pos);
    }
    return dir;
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (getline(ss, item, delim))
    {
        tokens.push_back(item);
    }
    return tokens;
}

class ShowMap
{
  public:
    static const int default_size = 1000; // max section count

    int update_cloud_count = 0;
    Eigen::Vector3f cur_posi;
    bool load_done = false;
    bool need_reload = false;
    ros::Subscriber sub_posi;
    ros::Subscriber reload_sub;
    ros::Subscriber sub_db_id;
    ros::Publisher cloudDso_pub;
    ros::Publisher painting_pub;
    ros::Publisher cloud_pub;
    ros::Publisher painting_fake_pub;
    ros::ServiceClient client;
    std::string painting_file;
    std::string pl_file;
    std::string dash_file;
    std::string solid_file;

    sensor_msgs::PointCloud paint_cloud;
    sensor_msgs::PointCloud dso_cloud;
    std::map<std::string, std::vector<std::vector<cv::Point3d> > > solidPointsMap;
    std::map<std::string, std::vector<std::vector<cv::Point3d> > > DashPointsMap;
    typedef std::map<std::string, std::vector<std::vector<cv::Point3d> > >::iterator MY_ITER;
    std::vector<std::vector<cv::Point3d> >* p_CurrentSolidVec = 0;
    std::vector<std::vector<cv::Point3d> >* p_CurrentDashVec = 0;

    std::string installDir;
    std::string res_root;

    ShowMap()
    {
        ros::NodeHandle n;

        sub_posi = n.subscribe<geometry_msgs::Point>("posi_show_map", 100, &ShowMap::update_posi, this);
        reload_sub = n.subscribe<std_msgs::String>("on_reload_map", 5000, &ShowMap::on_reload, this);
        cloudDso_pub = n.advertise<sensor_msgs::PointCloud>("pointcloudDso", 1);
        painting_pub = n.advertise<sensor_msgs::PointCloud>("painting", 1);
        cloud_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
        painting_fake_pub = n.advertise<visualization_msgs::Marker>("painting_fake", 1);
        //sub_db_id = n.subscribe<std_msgs::String>("section_id", 100, &ShowMap::on_db_id, this);

        client = n.serviceClient<show_map::algo>("algo_service");

        paint_cloud.header.frame_id = "/world";
        int max_sec_pointCount = 500;
        paint_cloud.points.reserve(max_sec_pointCount*default_size);


        dso_cloud.header.frame_id = "/world";
        dso_cloud.channels.resize(1);
        dso_cloud.channels[0].name = "rgb";

        //installDir = dirName(dirName(exePath()));
        //res_root = installDir + "/share/show_map/paints_res/";
        res_root = LocContext::getInstance()->getPaintDir();

        std::cout << "res_root = " << res_root << std::endl;
    }

    void on_db_id(const std_msgs::String::ConstPtr &msg)
    {
        // detect db id, load painting accoordingly
        std::cout << "section id: " << msg->data << std::endl;
        painting_file = msg->data + ".txt";
        std::cout << "detected painting: " << painting_file << std::endl;

        // load painting subjectively
        load_painting(painting_file.c_str());
        std::cout << "load painting done." << std::endl
                  << std::endl;
    }

    void load_all_painting(const char *painting_dir)
    {
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir(painting_dir)) != NULL)
        {
            /* print all the files and directories within directory */
            while ((ent = readdir(dir)) != NULL)
            {
                //printf ("%s\n", ent->d_name);
                std::string fullpath(painting_dir);
                fullpath += "/";
                fullpath += ent->d_name;
                load_painting(fullpath.c_str());
            }
            closedir(dir);
        }
        else
        {
            /* could not open directory */
            perror("");
            return;
        }
    }

    void load_painting(const char *painting_file)
    {
        need_reload = true;
        std::cout << "painting addr:" << painting_file << std::endl;
        Eigen::Matrix3d r_cb;
        r_cb << 0, 0, 1,
            -1, 0, 0,
            0, 1, 0;

        std::ifstream paintFile(painting_file);
        cv::Point3d refref_posi;
        bool firstRef = false;
        bool isDash = false;
        int pointCount = -1;
        //        int downSampleRate = 8;
        int downSampleRate = 1;
        int index = -1;

        paint_cloud.points.clear();
        solidPointsMap.clear();
        DashPointsMap.clear();
        if (paintFile.is_open())
        {
            char newseq_c[256];
            paintFile.getline(newseq_c, 256); //newseq token//newseq
            while (true)
            {
                char seqid_c[256];
                paintFile.getline(seqid_c, 256);
                std::cout << "new section, id = " << seqid_c << std::endl;
                if(solidPointsMap.find(seqid_c) == solidPointsMap.end()) {
                    solidPointsMap[seqid_c] = std::vector < std::vector < cv::Point3d > > ();
                    p_CurrentSolidVec = &solidPointsMap[seqid_c];
                }
                else
                    p_CurrentSolidVec = &solidPointsMap[seqid_c];

                if(DashPointsMap.find(seqid_c) == DashPointsMap.end()) {
                    DashPointsMap[seqid_c] = std::vector < std::vector < cv::Point3d > > ();
                    p_CurrentDashVec = &DashPointsMap[seqid_c];
                }
                else
                    p_CurrentDashVec = &DashPointsMap[seqid_c];

                char seqref_c[256];
                paintFile.getline(seqref_c, 256); //[0,0,0]
                cv::Point3d ref_posi;
                sscanf(seqref_c, "[%lf, %lf, %lf]", &ref_posi.x, &ref_posi.y, &ref_posi.z);
                std::cout<<seqref_c<<std::endl;
                if (firstRef)
                {
                    show_map::algo algo_srv;
                    algo_srv.request.absGPS.x = refref_posi.x;
                    algo_srv.request.absGPS.y = refref_posi.y;
                    algo_srv.request.absGPS.z = refref_posi.z;
                    algo_srv.request.refGPS.x = ref_posi.x;
                    algo_srv.request.refGPS.y = ref_posi.y;
                    algo_srv.request.refGPS.z = ref_posi.z;
                    client.call(algo_srv);
                    ref_posi.x = algo_srv.response.relGPS.x;
                    ref_posi.y = algo_srv.response.relGPS.y;
                    ref_posi.z = algo_srv.response.relGPS.z;
                    //std::cout<<"ref_ref_posi: "<<refref_posi<<std::endl;
                    //std::cout<<"rel_ref_posi: "<<ref_posi<<std::endl;
                }
                else
                {
                    refref_posi = ref_posi;
                    firstRef = true;
                    ref_posi.x = 0;
                    ref_posi.y = 0;
                    ref_posi.z = 0;
                }

                paintFile.getline(seqref_c, 256); //newline token//newline
                bool isEndfile = false;
                while (true)
                {
                    char linetype_c[256];
                    paintFile.getline(linetype_c, 256); //solid or dashed
                    isDash = !(linetype_c[0] == 's' && linetype_c[1] == 'o' && linetype_c[2] == 'l');

                    bool isEndSeq = false;
                    bool isEndLine = false;
                    std::vector<cv::Point3d> solidLineVec;
                    std::vector<cv::Point3d> DashLineVec;
                    while (true)
                    {
                        char linePosi[256];
                        paintFile.getline(linePosi, 256); //position
                        if (linePosi[3] == 'l' && linePosi[4] == 'i' && linePosi[5] == 'n') //newline token//newline
                        {
                            isEndLine = true;
                            break;
                        }
                        if (linePosi[3] == 's' && linePosi[4] == 'e' && linePosi[5] == 'q')
                        {
                            isEndSeq = true;
                            break;
                        }
                        if (linePosi[0] == 0)
                        {
                            isEndfile = true;
                            break;
                        }
                        //std::cout<<linePosi<<std::endl;
                        float x, y, z;
                        sscanf(linePosi, "%f %f %f", &x, &z, &y);
                        Eigen::Vector3d posi(ref_posi.x + x, ref_posi.y + y, ref_posi.z + z);
                        //Eigen::Vector3f posi(x,y,z);
                        posi = r_cb * posi;

                        pointCount++;
                        if (pointCount % downSampleRate == 0)
                        {
                            cv::Point3d pos;
                            pos.x = posi.x();
                            pos.y = posi.y();
                            pos.z = posi.z();
                            geometry_msgs::Point32 t_posi;
                            t_posi.x = posi.x();
                            t_posi.y = posi.y();
                            t_posi.z = posi.z();
                            //if(t_posi.x == 0 && t_posi.y == 0 && t_posi.z == 0)
                                //std::cout<<"which section: " << seqid_c << " isDash: " << isDash << " which point:  " << pointCount << std::endl;

                            paint_cloud.points.push_back(t_posi);
                            if (isDash)
                            {
                                DashLineVec.push_back(pos);
                            }
                            else
                            {
                                solidLineVec.push_back(pos);
                            }
                        }
                    }
                    if(isEndLine)
                    {
                        if(solidLineVec.size() > 0 && p_CurrentSolidVec)
                            p_CurrentSolidVec->push_back(solidLineVec);
                        if(DashLineVec.size() > 0 && p_CurrentDashVec)
                            p_CurrentDashVec->push_back(DashLineVec);

                        solidLineVec.clear();
                        DashLineVec.clear();
                    }
                    if (isEndSeq || isEndfile)
                    {
                        break;
                    }
                }
                if (isEndfile)
                {
                    load_done = true;
                    painting_pub.publish(paint_cloud);
                    std::cout<<"load painting done"<<std::endl;
                    break;
                }
            }
        }
        else
        {
            std::cout << "open painting file error!" << std::endl;
            return;
        }
    }

    void load_semiDense(const char *dsoClound_file)
    {
        std::cout << "semiDense addr:" << dsoClound_file << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDso_data (new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(dsoClound_file , *cloudDso_data) < 0){
            std::cout<<"no cloud file!!"<<std::endl;
            //return;
        }else{
            std::cout<<"load dso points: "<<cloudDso_data->size()<<std::endl;
        }
        Eigen::Matrix3f r_cb;
        r_cb << 0, 0, 1,
                -1, 0, 0,
                0, 1, 0;

        dso_cloud.points.clear();
        dso_cloud.channels[0].values.clear();

        for(int i=0; i<cloudDso_data->size();i++)
        {
            Eigen::Vector3f posi(cloudDso_data->points[i].x, cloudDso_data->points[i].y, cloudDso_data->points[i].z);

            posi = r_cb*posi;
//                     if((posi-cur_posi).norm()>250){
//                         continue;
//                     }
            geometry_msgs::Point32 t_posi;
            t_posi.x= posi.x();
            t_posi.y= posi.y();
            t_posi.z= posi.z();
            dso_cloud.points.push_back(t_posi);
            Eigen::Vector3i color_t = cloudDso_data->points[i].getRGBVector3i ();
            color_t=color_t*5;
            color_t[0] = (color_t[0] > 255) ? 255 : color_t[0];
            color_t[1] = (color_t[1] > 255) ? 255 : color_t[1];
            color_t[2] = (color_t[2] > 255) ? 255 : color_t[2];
            int rgb = color_t[0]*256*256+color_t[1]*256+color_t[2];
            float float_rgb = *reinterpret_cast<float*>(&rgb);
            dso_cloud.channels[0].values.push_back(float_rgb);
        }

        load_done = true;
        cloudDso_pub.publish(dso_cloud);
        std::cout<<"load semiDense done"<<std::endl;
    }

    void publish_marker()
    {
        int index = -1;
        for (MY_ITER it = solidPointsMap.begin(); it != solidPointsMap.end(); ++it)
        {
            std::vector < std::vector < cv::Point3d > >& solidVec = it->second;
            std::vector < std::vector < cv::Point3d > >& DashVec = DashPointsMap[it->first];

            for(int i = 0; i < solidVec.size(); ++i) {
                visualization_msgs::Marker roadSolidPaint;
                roadSolidPaint.header.frame_id = "/world";
                roadSolidPaint.header.stamp = ros::Time::now();
                roadSolidPaint.ns = "roadSolidPaint";
                roadSolidPaint.id = ++index;
                roadSolidPaint.type = visualization_msgs::Marker::LINE_STRIP;
                roadSolidPaint.action = visualization_msgs::Marker::ADD;
                roadSolidPaint.pose.position.x = 0;
                roadSolidPaint.pose.position.y = 0;
                roadSolidPaint.pose.position.z = 0;
                roadSolidPaint.pose.orientation.x = 0;
                roadSolidPaint.pose.orientation.y = 0;
                roadSolidPaint.pose.orientation.z = 0;
                roadSolidPaint.pose.orientation.w = 1;
                roadSolidPaint.scale.x = 0.2;
                roadSolidPaint.scale.y = 0.2;
                roadSolidPaint.scale.z = 0.2;
                roadSolidPaint.color.r = 0.46f;
                roadSolidPaint.color.g = 0.58f;
                roadSolidPaint.color.b = 0.86f;
                roadSolidPaint.color.a = 1;
                roadSolidPaint.lifetime = ros::Duration();

                for(int j = 0; j < solidVec[i].size(); ++j) {

                    geometry_msgs::Point point;
                    point.x = solidVec[i][j].x;
                    point.y = solidVec[i][j].y;
                    point.z = solidVec[i][j].z;
                    roadSolidPaint.points.push_back(point);
                }
                painting_fake_pub.publish(roadSolidPaint);
            }

            for(int i = 0; i < DashVec.size(); ++i) {
                visualization_msgs::Marker roadDashPaint;
                roadDashPaint.header.frame_id = "/world";
                roadDashPaint.header.stamp = ros::Time::now();
                roadDashPaint.ns = "roadDashPaint";
                roadDashPaint.id = ++index;
                roadDashPaint.type = visualization_msgs::Marker::LINE_LIST;
                roadDashPaint.action = visualization_msgs::Marker::ADD;
                roadDashPaint.pose.position.x = 0;
                roadDashPaint.pose.position.y = 0;
                roadDashPaint.pose.position.z = 0;
                roadDashPaint.pose.orientation.x = 0;
                roadDashPaint.pose.orientation.y = 0;
                roadDashPaint.pose.orientation.z = 0;
                roadDashPaint.pose.orientation.w = 1;
                roadDashPaint.scale.x = 0.2;
                roadDashPaint.scale.y = 0.2;
                roadDashPaint.scale.z = 0.2;
                roadDashPaint.color.r = 0.46f;
                roadDashPaint.color.g = 0.58f;
                roadDashPaint.color.b = 0.86f;
                roadDashPaint.color.a = 1;
                roadDashPaint.lifetime = ros::Duration();

                // make sure line_list has even number of points
                int pointNum = DashVec[i].size();
                if(pointNum % 2 == 1)
                    pointNum--;
                for(int j = 0; j < pointNum; ++j) {

                    geometry_msgs::Point point;
                    point.x = DashVec[i][j].x;
                    point.y = DashVec[i][j].y;
                    point.z = DashVec[i][j].z;
                    roadDashPaint.points.push_back(point);
                }
                painting_fake_pub.publish(roadDashPaint);
            }

        }
    }

    void publish_cloud()
    {
        painting_pub.publish(paint_cloud);

        static int update_cloud_count = 5000;
        update_cloud_count++;
        if(update_cloud_count>5000) {
            update_cloud_count = 0;
            cloudDso_pub.publish(dso_cloud);
        }
    }

    void clear_cloud()
    {
        paint_cloud.points.clear();
        painting_pub.publish(paint_cloud);
        dso_cloud.points.clear();
        cloudDso_pub.publish(dso_cloud);
    }

    void show()
    {
        while (ros::ok())
        {
            ros::Rate r(0.5);
            ros::spinOnce();

            if (!load_done){
                r.sleep();
                continue;
            }

            //publish_cloud();
            //publish_marker();

            ros::spinOnce();
            r.sleep();
        }
    }

    void update_posi(const geometry_msgs::Point::ConstPtr &msg)
    {
        update_cloud_count++;
        cur_posi.x() = msg->x;
        cur_posi.y() = msg->y;
        cur_posi.z() = msg->z;
    }

    void on_reload(const std_msgs::String::ConstPtr &sign)
    {
        std::string msg = sign->data;
        if(msg.empty())
            return;

        if(msg == "deleteAll")
        {
            clear_cloud();
            return;
        }
        std::vector<std::string> files = split(sign->data, ',');
        if(files.size() != 4)
            return;

        pl_file = files[0];
        painting_file = files[1];
        dash_file = files[2];
        solid_file = files[3];

        std::string cloud_dir=res_root + pl_file;
        std::string paint_dir=res_root + painting_file;
        load_semiDense(cloud_dir.c_str());
        load_painting(paint_dir.c_str());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_map");
    ShowMap myMap;

    myMap.show();
    return 0;
}
