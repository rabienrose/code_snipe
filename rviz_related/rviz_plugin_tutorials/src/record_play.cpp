/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <qt4/QtGui/QCheckBox>
#include <qt4/QtGui/QPushButton>
#include <qt4/QtGui/QLabel>
#include <qt4/QtGui/QPixmap>

#include <QTextEdit>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QDialog>
#include <QFileDialog>
#include <QTimer>
#include <QGroupBox>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "record_play.h"
#include "context.h"
#include "remoteConHelper.h"
#include "teleop_panel.h"

// other lib header files.

#include <Eigen/Eigen>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace rviz_plugin_tutorials
{
void DrawTransRec(cv::Mat &img, int x, int y, int width, int height, cv::Scalar color, double alpha)
{
    IplImage img_i(img);
    IplImage *rec = cvCreateImage(cvSize(width, height), img_i.depth, img_i.nChannels);
    cvRectangle(rec, cvPoint(0, 0), cvPoint(width, height), color, -1);
    cvSetImageROI(&img_i, cvRect(x, y, width, height));
    cvAddWeighted(&img_i, alpha, rec, 1 - alpha, 0.0, &img_i);
    cvResetImageROI(&img_i);
    return;
}
static void addFrame(QGroupBox *groupBox)
{
    groupBox->setStyleSheet("QGroupBox{border:1px solid gray;border-radius:2px;margin-top: 1ex;} "
                                    "QGroupBox::title{subcontrol-origin: margin;subcontrol-position:top center;padding:0 1px;}");
}
record_play::record_play(QWidget *parent)
    : rviz::Panel(parent)
{
    gridLayout = new QGridLayout;

    std::string user_name = remoteConHelper::getInst()->do_cmd("echo `whoami`", false, true)[0];
    ubuntu_rosbag_folder = "/home/" + user_name + "/Documents/RosBag_Folder";
    std::cout << "ubuntu_rosbag_folder:" << ubuntu_rosbag_folder << std::endl;

    sub_img = nr_.subscribe<record_msg::img_record>("/img_record", 100, &record_play::on_msg_image, this);
    sub_imu = nr_.subscribe<record_msg::imu_pack>("/imu_pack", 100, &record_play::on_msg_imu, this);
    sub_gps = nr_.subscribe<record_msg::gps_record>("/gps_record", 100, &record_play::on_msg_gps, this);

    QTimer *output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(sensor_check()));
    output_timer->start(1000);

    std::cout << "icon dir = " << LocContext::getInstance()->getIconDir() << std::endl;

    cv::Mat test_img = cv::imread((LocContext::getInstance()->getIconDir() + "/test_img.jpg").c_str());
    QImage img = remoteConHelper::getInst()->cvMat2QImage(test_img);
    labelimg = new QLabel();
    labelimg->setPixmap(QPixmap::fromImage(img).scaled(400, 250));
         labelimg->setScaledContents(true);
    show_img(0, 1000, test_img);

    QLabel *Camera_state = new QLabel(QWidget::tr("IMG:"));
    QLabel *GPS_state = new QLabel(QWidget::tr("GPS:"));
    QLabel *IMU_state = new QLabel(QWidget::tr("IMU:"));

    GPS_ImageLabel = new QLabel(this);
    IMU_ImageLabel = new QLabel(this);
    Camera_ImageLabel = new QLabel(this);

    GPS_ImageLabel->setScaledContents(true);
    IMU_ImageLabel->setScaledContents(true);
    Camera_ImageLabel->setScaledContents(true);

    red_pixmap.load((LocContext::getInstance()->getIconDir() + "/r.png").c_str());
    green_pixmap.load((LocContext::getInstance()->getIconDir() + "/g.png").c_str());
    yellow_pixmap.load((LocContext::getInstance()->getIconDir() + "/y.png").c_str());

    //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));

    GPS_ImageLabel->setMaximumHeight(30);
    GPS_ImageLabel->setMaximumWidth(30);
    GPS_ImageLabel->setPixmap(red_pixmap);

    IMU_ImageLabel->setMaximumHeight(30);
    IMU_ImageLabel->setMaximumWidth(30);
    IMU_ImageLabel->setPixmap(red_pixmap);

    Camera_ImageLabel->setMaximumHeight(30);
    Camera_ImageLabel->setMaximumWidth(30);
    Camera_ImageLabel->setPixmap(red_pixmap);

    QLabel *Label_Lat_info = new QLabel(QWidget::tr("Lat:"));
    QLabel *Label_Lon_info = new QLabel(QWidget::tr("Lon:"));
    QLabel *Label_Alt_info = new QLabel(QWidget::tr("Alt:"));
    GPS_Value_Lat = new QLabel();
    GPS_Value_Lon = new QLabel();
    GPS_Value_Alt = new QLabel();

    GPS_Value_Lat->setText(QString::fromStdString(remoteConHelper::getInst()->intToStr(gps_Lat)));
    GPS_Value_Lon->setText(QString::fromStdString(remoteConHelper::getInst()->intToStr(gps_Lon)));
    GPS_Value_Alt->setText(QString::fromStdString(remoteConHelper::getInst()->intToStr(gps_Alt)));

    QLabel *Label_record_info = new QLabel(QWidget::tr("Record:"));
    Button_Record_Start = new QPushButton;
    Button_Record_Start->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Record_Start, SIGNAL(clicked()), this, SLOT(Button_Record_Start_event()));

    Button_Record_Stop = new QPushButton;
    Button_Record_Stop->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Record_Stop, SIGNAL(clicked()), this, SLOT(Button_Record_Stop_event()));

    QLabel *Run_sensor_manage_info = new QLabel(QWidget::tr("Run_sensor_manage:"));
    Button_Run_sensor_manage = new QPushButton;
    Button_Run_sensor_manage->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Run_sensor_manage, SIGNAL(clicked()), this, SLOT(Button_Run_sensor_manage_event()));

    Button_Stop_sensor_manage = new QPushButton;
    Button_Stop_sensor_manage->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Stop_sensor_manage, SIGNAL(clicked()), this, SLOT(Button_Stop_sensor_manage_event()));

    QLabel *Label_play_info = new QLabel(QWidget::tr("Play:"));
    Button_Folder = new QPushButton;
    Button_Folder->setText("Choose_Folder");
    connect(Button_Folder, SIGNAL(clicked()), this, SLOT(Button_Folder_event()));

    ComboBox_Bag = new QComboBox;
    ComboBox_Bag->addItem(QWidget::tr("Choose_Bag")); //                                            "));
    connect(ComboBox_Bag, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Bag_valueChanged()));

    Button_Play_Start = new QPushButton;
    Button_Play_Start->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Play_Start, SIGNAL(clicked()), this, SLOT(Button_Play_Start_event()));

    Button_Play_Stop = new QPushButton;
    Button_Play_Stop->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Play_Stop, SIGNAL(clicked()), this, SLOT(Button_Play_Stop_event()));

    ComboBox_Speed = new QComboBox;
    ComboBox_Speed->addItem(QWidget::tr("Choose_Speed"));
    ComboBox_Speed->addItem(QWidget::tr("1"));
    ComboBox_Speed->addItem(QWidget::tr("2"));
    ComboBox_Speed->addItem(QWidget::tr("4"));
    ComboBox_Speed->addItem(QWidget::tr("8"));
    ComboBox_Speed->addItem(QWidget::tr("16"));
    ComboBox_Speed->addItem(QWidget::tr("32"));
    connect(ComboBox_Speed, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Speed_valueChanged()));

    //     Play_start_sec = new QTextEdit;
    //     Play_start_sec->resize(5,5);

    QLabel *Record_loc_info = new QLabel(QWidget::tr("Record_loc_info:"));
    QPushButton *Button_Record_loc_info_Start = new QPushButton;
    Button_Record_loc_info_Start->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Record_loc_info_Start, SIGNAL(clicked()), this, SLOT(Button_Record_Loc_info_Start_event()));

    QPushButton *Button_Record_loc_info_Stop = new QPushButton;
    Button_Record_loc_info_Stop->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Record_loc_info_Stop, SIGNAL(clicked()), this, SLOT(Button_Record_Loc_info_Stop_event()));

#ifndef DEV_BUILD
    QHBoxLayout *sh1 = new QHBoxLayout;
    sh1->addWidget(Camera_state);
    sh1->addWidget(Camera_ImageLabel);
    QHBoxLayout *sh2 = new QHBoxLayout;
    sh2->addWidget(IMU_state);
    sh2->addWidget(IMU_ImageLabel);
    QHBoxLayout *sh3 = new QHBoxLayout;
    sh3->addWidget(GPS_state);
    sh3->addWidget(GPS_ImageLabel);
    QHBoxLayout *sh4 = new QHBoxLayout;
    sh4->addWidget(Label_Lat_info);
    sh4->addWidget(GPS_Value_Lat);
    QHBoxLayout *sh5 = new QHBoxLayout;
    sh5->addWidget(Label_Lon_info);
    sh5->addWidget(GPS_Value_Lon);
    QHBoxLayout *sh6 = new QHBoxLayout;
    sh6->addWidget(Label_Alt_info);
    sh6->addWidget(GPS_Value_Alt);

    QVBoxLayout *sv1 = new QVBoxLayout;
    sv1->addLayout(sh1);
    sv1->addLayout(sh2);
    sv1->addLayout(sh3);
    sv1->addLayout(sh4);
    sv1->addLayout(sh5);
    sv1->addLayout(sh6);
    sv1->addStretch();

    QGroupBox *groupBoxRecordPlay = new QGroupBox(tr("Record/Play"));
    QHBoxLayout *h1 = new QHBoxLayout;
    h1->addWidget(Label_record_info);
    h1->addWidget(Button_Record_Start);
    h1->addWidget(Button_Record_Stop);

    QHBoxLayout *h2 = new QHBoxLayout;
    h2->addWidget(Record_loc_info);
    h2->addWidget(Button_Record_loc_info_Start);
    h2->addWidget(Button_Record_loc_info_Stop);

    QHBoxLayout *h3 = new QHBoxLayout;
    h3->addWidget(Label_play_info);
    h3->addWidget(Button_Folder);
    h3->addWidget(ComboBox_Bag);
    h3->addWidget(Button_Play_Start);
    h3->addWidget(Button_Play_Stop);
    h3->addWidget(ComboBox_Speed);

    QVBoxLayout *v1 = new QVBoxLayout;
    v1->addLayout(h1);
    v1->addLayout(h2);
    v1->addLayout(h3);
    groupBoxRecordPlay->setLayout(v1);
    addFrame(groupBoxRecordPlay);

    QGroupBox *groupBoxSensor = new QGroupBox(tr("Sensor Manage"));
    QHBoxLayout *ss1 = new QHBoxLayout;
    ss1->addWidget(Run_sensor_manage_info);
    ss1->addWidget(Button_Run_sensor_manage);
    ss1->addWidget(Button_Stop_sensor_manage);
    groupBoxSensor->setLayout(ss1);
    addFrame(groupBoxSensor);

    QHBoxLayout *il =new QHBoxLayout;
    il->addWidget(labelimg);
    il->addLayout(sv1);

    QVBoxLayout *fullLayout = new QVBoxLayout;
    fullLayout->addLayout(il);
    fullLayout->addWidget(groupBoxRecordPlay);
    fullLayout->addWidget(groupBoxSensor);
    setLayout(fullLayout);

#else
    QLabel *Debug_info = new QLabel(QWidget::tr("Debug_info:"));
    gridLayout->setMargin(5);  //表示控件与窗体的左右边距
    gridLayout->setSpacing(5); //设置组件的间隔为10像素
    gridLayout->setVerticalSpacing(5);

    gridLayout->setColumnStretch(0, 0.4); //表示第0列的比例
                                          //     gridLayout->setColumnStretch(1, 0.7);//表示第1列的比例
                                          //     gridLayout->setColumnStretch(2, 0.7);//表示第0列的比例
                                          //     gridLayout->setColumnStretch(3, 0.7);//表示第1列的比例
                                          //     gridLayout->setColumnStretch(4, 0.7);//表示第1列的比例

    gridLayout->addWidget(labelimg, 0, 0, 7, 5);

    gridLayout->addWidget(Camera_state, 0, 5, Qt::AlignRight);
    gridLayout->addWidget(Camera_ImageLabel, 0, 6);
    gridLayout->addWidget(IMU_state, 1, 5, Qt::AlignRight);
    gridLayout->addWidget(IMU_ImageLabel, 1, 6);
    gridLayout->addWidget(GPS_state, 2, 5, Qt::AlignRight);
    gridLayout->addWidget(GPS_ImageLabel, 2, 6);

    gridLayout->addWidget(Label_Lat_info, 3, 5, Qt::AlignRight);
    gridLayout->addWidget(Label_Lon_info, 4, 5, Qt::AlignRight);
    gridLayout->addWidget(Label_Alt_info, 5, 5, Qt::AlignRight);
    gridLayout->addWidget(GPS_Value_Lat, 3, 6, 1, 2);
    gridLayout->addWidget(GPS_Value_Lon, 4, 6, 1, 2);
    gridLayout->addWidget(GPS_Value_Alt, 5, 6, 1, 2);

    gridLayout->addWidget(Label_record_info, 7, 0, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Record_Start, 7, 1, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Record_Stop, 7, 2, 1, 1, Qt::AlignLeft);

    gridLayout->addWidget(Run_sensor_manage_info, 7, 3, 1, 2, Qt::AlignRight);
    gridLayout->addWidget(Button_Run_sensor_manage, 7, 5, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Stop_sensor_manage, 7, 6, 1, 1, Qt::AlignLeft);

    gridLayout->addWidget(Label_play_info, 8, 0, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Folder, 8, 1, 1, 2, Qt::AlignLeft);
    gridLayout->addWidget(ComboBox_Bag, 8, 3, 1, 2, Qt::AlignLeft);
    gridLayout->addWidget(Button_Play_Start, 8, 5, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Play_Stop, 8, 6, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(ComboBox_Speed, 9, 3, 1, 2, Qt::AlignLeft);

    gridLayout->addWidget(Record_loc_info, 9, 0, Qt::AlignLeft);
    gridLayout->addWidget(Button_Record_loc_info_Start, 9, 1, 1, 1, Qt::AlignLeft);
    gridLayout->addWidget(Button_Record_loc_info_Stop, 9, 2, 1, 1, Qt::AlignLeft);

    //     gridLayout->addWidget(Debug_info,10,0,Qt::AlignLeft);

    setLayout(gridLayout);
#endif

    //     Button_Folder_event();
}
void record_play::sensor_check()
{
    cur_time = ros::Time::now();
    //     std::cout<<"cur_time:"<<cur_time.toSec()<<std::endl;
    //     std::cout<<"imu_time:"<<imu_time.toSec()<<std::endl;
    //     std::cout<<"gps_time:"<<gps_time.toSec()<<std::endl;
    //     std::cout<<"img_time:"<<img_time.toSec()<<std::endl;
    if ((cur_time - img_time).toSec() > 1)
    {
        //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(red_pixmap);
        //gridLayout->addWidget(Camera_ImageLabel, 0, 6);
    }
    if ((cur_time - imu_time).toSec() > 1)
    {
        //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        IMU_ImageLabel->setPixmap(red_pixmap);
        //gridLayout->addWidget(IMU_ImageLabel, 1, 6);
    }
    if ((cur_time - gps_time).toSec() > 2)
    {
        //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(red_pixmap);
        //gridLayout->addWidget(GPS_ImageLabel, 2, 6);
    }
    if ((cur_time - img_time).toSec() > 2 && (cur_time - imu_time).toSec() > 2 && (cur_time - gps_time).toSec() > 2)
    {
        flag_bag_play = false;
    }
}
void record_play::on_msg_imu(const record_msg::imu_pack::ConstPtr &msg)
{
    flag_bag_play = true;
    imu_time = ros::Time::now();
    //IMU_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
    IMU_ImageLabel->setPixmap(green_pixmap);
    //gridLayout->addWidget(IMU_ImageLabel, 1, 6);
}
void record_play::on_msg_gps(const record_msg::gps_record::ConstPtr &msg)
{
    flag_bag_play = true;
    gps_time = ros::Time::now();
    double Lat = msg->gps.x * 90.0 / (1024 * 1024 * 1024);
    double Lon = msg->gps.y * 90.0 / (1024 * 1024 * 1024);
    double Alt = msg->gps.z;
    if (0 == Lat || 0 == Lon || 0 == Alt)
    {
        //GPS_ImageLabel->setPixmap(yellow_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(yellow_pixmap);
    }
    else
    {
        //GPS_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(green_pixmap);
    }
    //gridLayout->addWidget(GPS_ImageLabel, 2, 6);
    GPS_Value_Lat->setText(QString::fromStdString(remoteConHelper::getInst()->doubleToStr(Lat)));
    GPS_Value_Lon->setText(QString::fromStdString(remoteConHelper::getInst()->doubleToStr(Lon)));
    GPS_Value_Alt->setText(QString::fromStdString(remoteConHelper::getInst()->doubleToStr(Alt)));
    //gridLayout->addWidget(GPS_Value_Lat, 3, 6);
    //gridLayout->addWidget(GPS_Value_Lon, 4, 6);
    //gridLayout->addWidget(GPS_Value_Alt, 5, 6);
}
void record_play::on_msg_image(const record_msg::img_record::ConstPtr &msg)
{
    flag_bag_play = true;
    img_time = ros::Time::now();
    if (false == msg->img_status)
    {
        //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(red_pixmap);
    }
    else
    {
        cv::Mat temp_img;
        temp_img = cv::imdecode(msg->vecCharImage, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
        //Camera_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(green_pixmap);
        //gridLayout->addWidget(Camera_ImageLabel, 0, 6);

        //         cv::imwrite("../data/jpg/" + remoteConHelper::getInst()->intToStr(img_id)+".jpg", temp_img);
        //         std::cout<<"frameId:"<<img_id<<std::endl;

        img_id++;
        switch_show_img++;
        //     std::cout<<"before opening bag file"<<std::endl;
        //     std::cout<<"bag name:"<<Bag_dir.toStdString() + "/" + bag_list[ComboBox_Bag_currentIndex-1]<<std::endl;
        //     std::string bagfile = Bag_dir.toStdString() + "/" + bag_list[ComboBox_Bag_currentIndex-1];
        //     rosbag::Bag bag;
        //     std::cout<<"opening bag file"<<std::endl;
        //     bag.open(bagfile,rosbag::bagmode::Read);
        //     std::cout<<"bag file has been opened"<<std::endl;
        //     rosbag::View view(bag, rosbag::TopicQuery("/img_record"));
        //
        //     std::cout<<"view.size():"<<view.size()<<std::endl;
        //     std::cout<<"msg->number:"<<msg->number<<std::endl;
        if (ceil((float)10 / ComboxBox_Speed) == switch_show_img)
        {
            show_img(img_id, 0, temp_img);
            switch_show_img = 0;
        }
    }
}
void record_play::show_img(int cur_id, int sum_id, cv::Mat cur_img)
{
    cv::resize(cur_img, cur_img, cv::Size(img_w, img_h));
    const cv::Scalar colorBlack(0, 0, 0);
    const cv::Scalar colorWhite(255, 255, 255);

    std::stringstream text;
    text << "cur_id:" << cur_id << "/" << sum_id;
    DrawTransRec(cur_img, 0, img_h - 30, 200, 30, colorBlack, 0.5);
    cv::putText(cur_img, text.str(), cv::Point2d(3, img_h - 5), cv::FONT_HERSHEY_TRIPLEX, 0.8, colorWhite, 1);
    text.str("");

    if (true == flag_record && true == flag_show_img)
    {
        text << "Record...";
        DrawTransRec(cur_img, 0, 0, 200, 30, colorBlack, 0.5);
        cv::putText(cur_img, text.str(), cv::Point2d(3, 25), cv::FONT_HERSHEY_TRIPLEX, 0.8, colorWhite, 1);
        text.str("");
    }
    flag_show_img = !flag_show_img;
    //     cv::cvtColor(cur_img, cur_img, CV_BGR2BGRA);
    //     cv::resize(cur_img,cur_img,cv::Size(cur_img.cols, cur_img.rows));
    //     sensor_msgs::Image rviz_img;
    //     rviz_img.height = cur_img.rows;
    //     rviz_img.width = cur_img.cols;
    //     rviz_img.encoding = "8UC4";
    //     unsigned char *p = cur_img.ptr<unsigned char>(0);
    //     std::vector<unsigned char> vec(p, p+cur_img.cols*4*cur_img.rows);
    //     rviz_img.data = vec;
    //     rviz_img.step = cur_img.cols*4;
    //     img_pub.publish(rviz_img);

    QImage img = remoteConHelper::getInst()->cvMat2QImage(cur_img);
    labelimg->setPixmap(QPixmap::fromImage(img).scaled(400, 250));
    //gridLayout->addWidget(labelimg, 0, 0, 7, 5);
    return;
}
void record_play::Button_Record_Loc_info_Stop_event()
{
    std::vector<std::string> re_lines = remoteConHelper::getInst()->do_cmd("rosnode list");
    for (int i = 0; i < re_lines.size(); i++)
    {
        if (re_lines[i][1] == 'r' && re_lines[i][2] == 'e' && re_lines[i][3] == 'c' &&
            re_lines[i][4] == 'o' && re_lines[i][5] == 'r' && re_lines[i][6] == 'd' &&
            re_lines[i][7] == '_')
        {
            std::stringstream ss;
            ss << "rosnode kill " << re_lines[i];
            re_lines = remoteConHelper::getInst()->do_cmd(ss.str());
            flag_record = false;
        }
    }
}
void record_play::Button_Record_Loc_info_Start_event()
{
    std::string cur_database_from_tele = rviz_plugin_tutorials::TeleopPanel::getInst()->cur_database;
    std::stringstream ss;
    ss << "rosbag record -o " << cur_database_from_tele
       << " predict_server update_server match_server gps_server image_server rviz_control database_server kfposition_server data_record";
    remoteConHelper::getInst()->do_cmd(ss.str(), false, false);
}

void record_play::Button_Record_Start_event()
{
    if (true == flag_bag_play)
    {
        if (access(ubuntu_rosbag_folder.c_str(), 0) == -1)
        {
            int flag = mkdir(ubuntu_rosbag_folder.c_str(), 0777);
            if (flag == 0)
            {
                std::cout << "mkdir rosbag_folder successfully" << std::endl;
            }
            else
            {
                std::cout << "mkdir rosbag_folder errorly" << std::endl;
            }
        }
        std::string Record_dir = ubuntu_rosbag_folder + "/Defult_folder";
        std::cout << "Record_dir:" << Record_dir << std::endl;
        if (access(Record_dir.c_str(), 0) == -1)
        {
            int flag = mkdir(Record_dir.c_str(), 0777);
            if (flag == 0)
            {
                std::cout << "mkdir Record_dir successfully" << std::endl;
            }
            else
            {
                std::cout << "mkdir Record_dir errorly" << std::endl;
            }
        }
        std::stringstream ss;
        ss << "cd " << Record_dir << "&& rosbag record "
           << "/gps_record "
           << "/img_record "
           << "/imu_pack";
        flag_record = true;
        remoteConHelper::getInst()->do_cmd(ss.str(), false, false);
    }
}
void record_play::Button_Record_Stop_event()
{
    if (true == flag_bag_play)
    {
        std::vector<std::string> re_lines = remoteConHelper::getInst()->do_cmd("rosnode list");
        for (int i = 0; i < re_lines.size(); i++)
        {
            if (re_lines[i][1] == 'r' && re_lines[i][2] == 'e' && re_lines[i][3] == 'c' &&
                re_lines[i][4] == 'o' && re_lines[i][5] == 'r' && re_lines[i][6] == 'd' &&
                re_lines[i][7] == '_')
            {
                std::stringstream ss;
                ss << "rosnode kill " << re_lines[i];
                re_lines = remoteConHelper::getInst()->do_cmd(ss.str());
                flag_record = false;
            }
        }
    }
}
void record_play::Button_Play_Start_event()
{
    Button_Play_Stop_event();
    switch_show_img = 0;
    std::stringstream ss;
    img_id = 0;
    std::string set_ros_ip_str = remoteConHelper::getInst()->set_ros_ip_str;
    ss << "rosbag play -q -r " << remoteConHelper::getInst()->intToStr(ComboxBox_Speed) << " " << Bag_dir.toStdString() << "/" << ComboBox_Bag_currentBag;
    remoteConHelper::getInst()->do_cmd(ss.str(), false, false);
}
void record_play::Button_Play_Stop_event()
{
    std::vector<std::string> re_lines = remoteConHelper::getInst()->do_cmd("rosnode list", false, true);
    for (int i = 0; i < re_lines.size(); i++)
    {
        if (re_lines[i][1] == 'p' && re_lines[i][2] == 'l' && re_lines[i][3] == 'a' &&
            re_lines[i][4] == 'y' && re_lines[i][5] == '_')
        {
            std::stringstream ss;
            ss << "rosnode kill " << re_lines[i];
            re_lines = remoteConHelper::getInst()->do_cmd(ss.str(), false, true);
        }
    }
}
void record_play::Button_Folder_event()
{
    //     QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Images (*.png *.xpm *.jpg)"));
    Bag_dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                QString::fromStdString(ubuntu_rosbag_folder),
                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    ubuntu_rosbag_folder = Bag_dir.toStdString();
    std::cout << "Bag_dir:" << Bag_dir.toStdString() << std::endl;

    std::stringstream ss;
    ss << "find " << Bag_dir.toStdString() << " -maxdepth 1 -type f -name \"*.bag\" | xargs -I {} basename '{}'";
    //     ss << "cd " << Bag_dir.toStdString() << " && find . ! -name \".\" -type d -prune -o -type f -name \"*.bag\" -print | xargs -I {} basename '{}'";
    std::vector<std::string> re = remoteConHelper::getInst()->do_cmd(ss.str(), false, true);
    for (int i = 0; i < re.size(); i++)
    {
        std::cout << "bag_name:" << re[i] << std::endl;
    }
    bag_list = re;
    ComboBox_Bag->clear();
    ComboBox_Bag->addItem(QWidget::tr("Choose_Bag"));
    for (int i = 0; i < bag_list.size(); i++)
        ComboBox_Bag->addItem(QWidget::tr(bag_list[i].c_str()));
}
void record_play::Button_Run_sensor_manage_event()
{
    std::stringstream ss;
    ss << "cd " << LocContext::getInstance()->getParam("Control", "dataReceiver_path") << " && "
       << "echo '" << LocContext::getInstance()->getParam("Control", "remote_password") << "'|sudo -S ./dataReceive.sh";
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
}
void record_play::Button_Stop_sensor_manage_event()
{
    std::stringstream ss;
    ss << "cd " << LocContext::getInstance()->getParam("Control", "dataReceiver_path") << " && "
       << "echo '" << LocContext::getInstance()->getParam("Control", "remote_password") << "'|sudo -S killall dataReceive";
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
}
void record_play::ComboBox_Bag_valueChanged()
{
    ComboBox_Bag_currentBag = ComboBox_Bag->currentText().toStdString();
}
void record_play::ComboBox_Speed_valueChanged()
{
    switch_show_img = 0;
    switch (ComboBox_Speed->currentIndex())
    {
    case 1:
        ComboxBox_Speed = 1;
        break;
    case 2:
        ComboxBox_Speed = 2;
        break;
    case 3:
        ComboxBox_Speed = 4;
        break;
    case 4:
        ComboxBox_Speed = 8;
        break;
    case 5:
        ComboxBox_Speed = 16;
        break;
    case 6:
        ComboxBox_Speed = 32;
        break;
    default:
        ComboxBox_Speed = 1;
    }
}
// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void record_play::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void record_play::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::record_play, rviz::Panel)
// END_TUTORIAL
