//
// Created by user on 1/2/18.
//

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
#include <string.h>

static char* trim(char *_newseq_c, char space_char = '\t')
{
    if (_newseq_c[0] == space_char)
    {
        // remove space
        int len = strlen(_newseq_c);
        int start = 0;
        int end = len - 1;
        for (; start < len && end > 0;)
        {
            if (*(_newseq_c + start) == space_char)
            {
                start++;
            }
            if (*(_newseq_c + end) == space_char)
            {
                end--;
            }

            if (*(_newseq_c + start) != space_char && (*(_newseq_c + end) != space_char))
                break;
        }
        *(_newseq_c + end + 1) = '\0';
        return _newseq_c + start;
        //std::cout << "removed space:\t" << _newseq_c << std::endl;
    }
    return _newseq_c;
}

/*Input: /home/user/workspace/loc/core/rdb-loc-visualization/ubuntu_demo/rviz_modify/src/show_map/paints_res/kml/CCDemo_paint.kml */
int main(int argc, char **argv)
{
    /* MPG */
    //    float absGPSX = -83.684987;
    //    float absGPSY = 42.595111;
    //    float absGPSZ = 0;

    /* CC Nissan */
    /*float absGPSX = -83.111793;
    float absGPSY = 42.4766622;
    float absGPSZ = 0;*/

    /*20180211*/
    float absGPSX = -83.080768;
    float absGPSY = 42.489736;
    float absGPSZ = 0;
    ros::init(argc, argv, "painting_cov");
    ros::NodeHandle n;
    ros::ServiceClient client;
    std::cout << "start converting " << argv[1] << std::endl;
    client = n.serviceClient<show_map::algo>("algo_service");

    if (argc == 5)
    {
        absGPSX = atof(argv[2]);
        absGPSY = atof(argv[3]);
        absGPSZ = atof(argv[4]);
        std::cout << "Anchor: " << absGPSX << ", " << absGPSY << ", " << absGPSZ << std::endl;
    }

    std::ifstream paintFile(argv[1]);
    cv::Point3d refref_posi;

    std::map<std::string, std::vector<std::vector<std::vector<cv::Point3d> > > > sectPosMap;
    typedef std::map<std::string, std::vector<std::vector<std::vector<cv::Point3d> > > >::iterator MY_ITER;
    std::vector<std::vector<std::vector<cv::Point3d> > > *p_CurrentPosVec = 0;
    std::vector<std::vector<cv::Point3d> >* p_CurrentSolidVec = 0;
    std::vector<std::vector<cv::Point3d> >* p_CurrentDashVec = 0;

    //char sectId[128]= {'\0'};
    char temp[1024] = {'\0'};
    uint64_t id = 0;

    if (paintFile.is_open())
    {
        std::cout << "open file success" << std::endl;
        int line_count = 0;

        bool isVisible = true;
        bool findPaint = false;

        int isPaint = 0; // 0--false, 1--solid, 2--dashed
        std::vector<cv::Point3d> lineVec;
        bool hasTab = false;
        while (ros::ok())
        {
            char _newseq_c[2048];
            paintFile.getline(_newseq_c, 2048);
            line_count++;

            if (_newseq_c[0] == '\0' || _newseq_c[0] == '#')
                continue;

            char *newseq_c = trim(_newseq_c);
            /* char *newseq_c = _newseq_c;
            if (_newseq_c[0] == '\t')
            {
                // remove space
                int len = strlen(_newseq_c);
                int start = 0;
                int end = len - 1;
                for (; start < len && end > 0;)
                {
                    if (*(_newseq_c + start) == '\t')
                    {
                        start++;
                    }
                    if (*(_newseq_c + end) == '\t')
                    {
                        end--;
                    }

                    if (*(_newseq_c + start) != '\t' && (*(_newseq_c + end) != '\t'))
                        break;
                }
                *(_newseq_c + end + 1) = '\0';
                newseq_c = _newseq_c + start;
                //std::cout << "removed space:\t" << newseq_c << std::endl;
            } */
            if (newseq_c[0] == '<')
            {
                if (!findPaint)
                {
                    if (0 == strncmp(newseq_c, "<name>14", 8))
                        findPaint = true;
                }
                if (0 == strncmp(newseq_c, "</kml", 5))
                {
                    break;
                }
                if (0 == strncmp(newseq_c, "<name>14", 8))
                {
                    sscanf(newseq_c, "<name>%lu<%s", &id, temp); // need use %lu to load, %s doesn't work
                    std::stringstream ss;
                    ss << id;
                    std::cout << "section id: " << ss.str() << std::endl;

                    if (sectPosMap.find(ss.str()) == sectPosMap.end())
                    {
                        sectPosMap[ss.str()] = std::vector<std::vector<std::vector<cv::Point3d> > >();
                        p_CurrentPosVec = &sectPosMap[ss.str()];
                        p_CurrentPosVec->resize(2);

                    }
                    else
                    {
                        p_CurrentPosVec = &sectPosMap[ss.str()];
                    }

                    std::vector<std::vector<std::vector<cv::Point3d> > >& posVec = *p_CurrentPosVec;
                    p_CurrentSolidVec = &posVec[0];
                    p_CurrentDashVec = &posVec[1];
                }
                    //                else if (0 != strstr(newseq_c, "<visibility>0</visibility>")) {
                    //                    //std::cout<<"line: " << line_count << ": " << "visibility 0" << std::endl;
                    //                    isVisible = false;
                    //                }
                    //else if (0 != strstr(newseq_c, "<visibility>1</visibility>")) {
                    //                else if (0 == strncmp(newseq_c, "<name>paint_", 12)) {
                    //                    //std::cout<<"line: " << line_count << ": " << "visibility 1" << std::endl;
                    //                    isVisible = true;
                    //                }
                else if (0 == strcmp(newseq_c + strlen(newseq_c) - 13, "_solid</name>") || 0 == strcmp(newseq_c + strlen(newseq_c) - 14, "_dashed</name>"))
                {
                    if (*(newseq_c + strlen(newseq_c) - 12) == 's')
                    {
                        isPaint = 1; // solid
                        std::cout << "found solid" << std::endl;
                    }
                    else if (*(newseq_c + strlen(newseq_c) - 13) == 'd')
                    {
                        isPaint = 2; // dashed
                        std::cout << "found dashed" << std::endl;
                    }

                    /* for (int i = 0; i < 12; ++i)
                    {
                        paintFile.getline(newseq_c, 2048);
                        line_count++;
                    } */
                    while(ros::ok()) {
                        paintFile.getline(newseq_c, 2048);
                        line_count++;
                        char *p = trim(newseq_c);
                        if(0 == strcmp(p + strlen(p) - 13, "<coordinates>")) {
                            lineVec.clear();
                            std::cout<<"line: " << line_count << ": " << "start coordinates" << std::endl;
                            break;
                        }
                    }
                }
                else if (0 == strncmp(newseq_c, "</coordinates>", 14))
                {
                    if(isPaint == 1) {
                        std::vector<std::vector<cv::Point3d> > &posVec = *p_CurrentSolidVec;
                        posVec.push_back(lineVec);
                    }
                    else if(isPaint == 2) {
                        std::vector<std::vector<cv::Point3d> > &posVec = *p_CurrentDashVec;
                        posVec.push_back(lineVec);
                    }
                    isPaint = 0;
                    std::cout<<"line: " << line_count << ": " << "end coordinates" << std::endl;
                }
            }

            if (findPaint && 0 != isPaint && p_CurrentPosVec)
            {

                cv::Point3d ref_posi;
                cv::Point3d rel_posi;

                while (ros::ok())
                {
                    char _newposi_c[204800];
                    paintFile.getline(_newposi_c, 204800);
                    line_count++;

                    if (_newposi_c[0] == '\0' || _newposi_c[0] == '#')
                        continue;

                    char *newposi_c = trim(_newposi_c);
                    //std::cout<<newposi_c<<std::endl;
                    /*char *newposi_c = _newposi_c;
                    if (_newposi_c[0] == '\t')
                    {
                        // remove space
                        int len = strlen(_newposi_c);
                        int start = 0;
                        int end = len - 1;
                        for (; start < len && end > 0;)
                        {
                            if (*(_newposi_c + start) == '\t')
                            {
                                start++;
                            }
                            if (*(_newposi_c + end) == '\t')
                            {
                                end--;
                            }

                            if (*(_newposi_c + start) != '\t' && (*(_newposi_c + end) != '\t'))
                                break;
                        }
                        *(_newposi_c + end + 1) = '\0';
                        newposi_c = _newposi_c + start;
                        //std::cout << "removed space:\t" << newposi_c;
                    }*/
                    if (newposi_c[0] == '<')
                    {
                        if (0 == strncmp(newposi_c, "</coordinates>", 14))
                        {
                            if(isPaint == 1) {
                                std::vector<std::vector<cv::Point3d> > &posVec = *p_CurrentSolidVec;
                                posVec.push_back(lineVec);
                            }
                            else if(isPaint == 2) {
                                std::vector<std::vector<cv::Point3d> > &posVec = *p_CurrentDashVec;
                                posVec.push_back(lineVec);
                            }
                            isPaint = 0;
                            std::cout<<"line: " << line_count << ": " << "end coordinates" << std::endl;
                        }
                        break;
                    }
                    if (isVisible)
                    {
                        if(newposi_c != _newposi_c) {
                            char *cursor = newposi_c;
                            char *end_cursor = cursor;
                            for (; *end_cursor != ' ' && *end_cursor != '\0'; ++end_cursor);

                            char coords[1024] = {'\0'};
                            strncpy(coords, cursor, end_cursor - cursor);
                            //std::cout << "coords:\t" << coords << std::endl;

                            while (*cursor != '\0') {
                                sscanf(coords, "%lf,%lf,%lf", &ref_posi.x, &ref_posi.y, &ref_posi.z);

                                show_map::algo algo_srv;

                                algo_srv.request.absGPS.x = absGPSX;
                                algo_srv.request.absGPS.y = absGPSY;
                                algo_srv.request.absGPS.z = absGPSZ;

                                algo_srv.request.refGPS.x = ref_posi.x;
                                algo_srv.request.refGPS.y = ref_posi.y;
                                algo_srv.request.refGPS.z = ref_posi.z;
                                //algo_srv.request.refGPS.z=1.5;//
                                client.call(algo_srv);
                                rel_posi.x = algo_srv.response.relGPS.x;
                                rel_posi.z = algo_srv.response.relGPS.y;
                                rel_posi.y = algo_srv.response.relGPS.z;
                                //std::cout<<"abs_posi: "<<ref_posi<<std::endl;
                                //std::cout<<"ref_posi: "<<ref_posi<<std::endl;
                                //std::cout<<"rel_posi: "<<rel_posi<<std::endl;

                                lineVec.push_back(rel_posi);

                                if (end_cursor == '\0')
                                    break;
                                cursor = end_cursor + 1;
                                if (cursor == '\0')
                                    break;
                                end_cursor = cursor;
                                for (; *end_cursor != ' ' && *end_cursor != '\0'; ++end_cursor);

                                strncpy(coords, cursor, end_cursor - cursor);
                                //std::cout << "coords:\t" << coords << std::endl;
                            }
                        } else{
                            sscanf(newposi_c, "%lf,%lf,%lf", &ref_posi.x, &ref_posi.y, &ref_posi.z);

                            show_map::algo algo_srv;

                            algo_srv.request.absGPS.x = absGPSX;
                            algo_srv.request.absGPS.y = absGPSY;
                            algo_srv.request.absGPS.z = absGPSZ;

                            algo_srv.request.refGPS.x = ref_posi.x;
                            algo_srv.request.refGPS.y = ref_posi.y;
                            algo_srv.request.refGPS.z = ref_posi.z;
                            //algo_srv.request.refGPS.z=1.5;//
                            client.call(algo_srv);
                            rel_posi.x = algo_srv.response.relGPS.x;
                            rel_posi.z = algo_srv.response.relGPS.y;
                            rel_posi.y = algo_srv.response.relGPS.z;
                            //std::cout<<"abs_posi: "<<ref_posi<<std::endl;
                            //std::cout<<"ref_posi: "<<ref_posi<<std::endl;
                            //std::cout<<"rel_posi: "<<rel_posi<<std::endl;

                            lineVec.push_back(rel_posi);
                        }
                    }
                }
            }
        }

        std::string kml_name(argv[1]);
        std::string::size_type suffix_pos = kml_name.find_last_of(".");
        std::string paint_name(kml_name.substr(0, suffix_pos) + ".txt");
        std::cout << paint_name << std::endl;
        std::ofstream out(paint_name.c_str());

        for (MY_ITER it = sectPosMap.begin(); it != sectPosMap.end(); ++it)
        {
            if (it->first.empty())
                continue;

            std::vector<std::vector<std::vector<cv::Point3d> > > &posVec = it->second;

            cv::Point3d refref_posi;
            out << "newseq" << std::endl;  // new section flag
            out << it->first << std::endl; // section id
            out << refref_posi << std::endl;

            if (posVec.size() != 2)
                continue;

            std::vector<std::vector<cv::Point3d> > pSolidVec = posVec[0];
            std::vector<std::vector<cv::Point3d> > pDashVec = posVec[1];

            for (int i = 0; i < pSolidVec.size(); ++i)
            {
                out << "newline" << std::endl;
                out << "solid" << std::endl;
                for(int j = 0; j < pSolidVec[i].size(); ++j) {
                    out << std::setprecision(16) << pSolidVec[i][j].x << " " << std::setprecision(16) << pSolidVec[i][j].y
                        << " " << std::setprecision(16) << pSolidVec[i][j].z << std::endl;
                    //std::cout << std::setprecision(16) << pSolidVec[i][j].x << " " << std::setprecision(16) << pSolidVec[i][j].y << " " << std::setprecision(16) << pSolidVec[i][j].z << std::endl;
                }
            }

            for (int i = 0; i < pDashVec.size(); ++i)
            {
                out << "newline" << std::endl;
                out << "dashed" << std::endl;
                for(int j = 0; j < pDashVec[i].size(); ++j) {
                    out << std::setprecision(16) << pDashVec[i][j].x << " " << std::setprecision(16) << pDashVec[i][j].y
                        << " " << std::setprecision(16) << pDashVec[i][j].z << std::endl;
                    //std::cout << std::setprecision(16) << pDashVec[i][j].x << " " << std::setprecision(16) << pDashVec[i][j].y << " " << std::setprecision(16) << pDashVec[i][j].z << std::endl;
                }
            }

        }

        out.close();
    }

    return 0;
}