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
#include <QSizePolicy>
#include <QDir>
#include <QFont>
#include <QTextBrowser>
#include <QButtonGroup>
#include <QGroupBox>
#include <QStackedWidget>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include "teleop_panel.h"
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
// other lib header files.

#include <Eigen/Eigen>
#include "context.h"
#include <time.h>

namespace rviz_plugin_tutorials
{
static void addFrame(QGroupBox *groupBox)
{
    groupBox->setStyleSheet("QGroupBox{border:1px solid gray;border-radius:2px;margin-top: 1ex;} "
                                    "QGroupBox::title{subcontrol-origin: margin;subcontrol-position:top center;padding:0 1px;}");
}
TeleopPanel::TeleopPanel(QWidget *parent)
    : rviz::Panel(parent), linear_velocity_(0), angular_velocity_(0)
{
    con = remoteConHelper::getInst();

    show_debug_info = 1;
    get_fist_gps_time_flag = false;
    button_start_all_flag = false;
    start_frameId_flag = 0;
    start_frameId = 0;
    img_miss_num = 0;
    img_false_num = 0;
    gps_miss_num = 0;
    gps_false_num = 0;
    imu_miss_num = 0;
    imu_false_num = 0;
    show_dgps_info = LocContext::getInstance()->getParam("Control", "switch_show_dgps") == "True" ? true : false;

    //ssh_ros_env = "export ROS_MASTER_URI=http://" + LocContext::getInstance()->getParam("Control", "remote_ip");
    // + ":11311/;export PYTHONPATH=/opt/ros/install_isolated/lib/python2.7/dist-packages;/opt/ros/install_isolated/bin/";

    std::string ROS_IP = LocContext::getInstance()->getParam("Control", "remote_ip");
    std::string ROS_MASTER_URI = std::string("http://") + ROS_IP + ":11311";
    std::stringstream ss;
    ss << "export ROS_IP=" << ROS_IP << " ; ";
    ss << "export ROS_MASTER_URI=" << ROS_MASTER_URI << " ; ";
    ss << "source /opt/ros/install_isolated/setup.bash ; ";
    ssh_ros_env = ss.str();

    cur_code = "/loc_algo_ros";
    cur_execute_to_kill = "LocRealTimeRosAlgo";

    const cv::Scalar colorBlack(0, 0, 0);
    const cv::Scalar colorRed(0, 0, 255);
    const cv::Scalar colorGreen(0, 255, 0);
    const cv::Scalar colorBlue(255, 0, 0);
    const cv::Scalar colorGrey(200, 200, 200);
    const cv::Scalar colorPurple(255, 0, 255);

    sub_current_uncertainty = nh_.subscribe<loc_server::loc_predict_msg>("predict_server", 100, &TeleopPanel::on_show_current_uncertainty, this);
    sub_sensor_state = nh_.subscribe<std_msgs::String>("sensor_heartbeat", 100, &TeleopPanel::on_sensor_status, this);
    sub_gps_record_time = nh_.subscribe<record_msg::gps_record>("gps_record", 1, &TeleopPanel::on_get_first_gps_time, this);
    sub_update = nh_.subscribe<loc_server::loc_update_msg>("update_server", 100, &TeleopPanel::on_update, this);
    sub_database = nh_.subscribe<sensor_msgs::PointCloud>("database_server", 100, &TeleopPanel::on_database, this);
    sub_rprj_err = nh_.subscribe<loc_server::reproj_err_msg>("rprj_err", 100, &TeleopPanel::on_show_rprj_err, this);
    sub_new_run = nh_.subscribe<std_msgs::String>("new_run", 100, &TeleopPanel::on_new_run, this);
    sub_gps = nh_.subscribe<geometry_msgs::Point32>("gps_server", 100, &TeleopPanel::on_gps, this);
    sub_img = nh_.subscribe<sensor_msgs::Image>("cur/image", 100, &TeleopPanel::on_img, this);
    db_ready_notice = nh_.subscribe<std_msgs::String>("db_ready_notice", 100, &TeleopPanel::on_db_ready_notice,this);

    paint_publisher_ = nh_.advertise<std_msgs::String>("on_reload_map", 1);
    rviz_control_publisher_ = nh_.advertise<loc_server::rviz_control_msg>("rviz_control", 1);
    publisher_stop_algo = nh_.advertise<std_msgs::String>("stop_algo_signal", 1);
    publisher_file_name = nh_.advertise<std_msgs::String>("file_name", 1);
    view_control_publisher_ = nh_.advertise<loc_server::view_control_msg>("view_control", 1);
    publisher_run_loc = nh_.advertise<std_msgs::String>("run_loc_signal", 1);
    publisher_Reinit_loc = nh_.advertise<std_msgs::String>("Reinit_loc_signal", 1);
    publisher_Quickinit_loc = nh_.advertise<std_msgs::String>("Quickinit_loc_signal", 1);

    QTimer *output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(sensor_check()));
    output_timer->start(1000);

    QLabel *Show_loc_match_info = new QLabel(QWidget::tr("Show_loc_match:"));
    QCheckBox *CheckBox_Loc = new QCheckBox;
    connect(CheckBox_Loc, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_Loc_onStateChanged(int)));

    QLabel *Show_upd_match_info = new QLabel(QWidget::tr("Show_upd_match:"));
    QCheckBox *CheckBox_Upd = new QCheckBox;
    connect(CheckBox_Upd, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_Upd_onStateChanged(int)));

    QLabel *Show_frameId_info = new QLabel(QWidget::tr("Show_frameId:"));
    QCheckBox *CheckBox_FrameId = new QCheckBox;
    connect(CheckBox_FrameId, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_FrameId_onStateChanged(int)));

    QLabel *Show_links_info = new QLabel(QWidget::tr("Show_map_links:"));
    CheckBox_Links = new QCheckBox;
    connect(CheckBox_Links, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_Links_onStateChanged(int)));

    QLabel *Show_his_uncety_info = new QLabel(QWidget::tr("Show_his_uncety:"));
    CheckBox_His_uncety = new QCheckBox;
    connect(CheckBox_His_uncety, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_His_uncety_onStateChanged(int)));

    QLabel *Show_rprj_err_info = new QLabel(QWidget::tr("Show_rprj_err:"));
    QCheckBox *CheckBox_rprj_err = new QCheckBox;
    connect(CheckBox_rprj_err, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_rprj_err_onStateChanged(int)));

    QLabel *Camera_state = new QLabel(QWidget::tr("IMG:"));
    QLabel *GPS_state = new QLabel(QWidget::tr("GPS:"));
    QLabel *IMU_state = new QLabel(QWidget::tr("IMU:"));

    IMG_MISS = new QLabel();
    GPS_MISS = new QLabel();
    IMU_MISS = new QLabel();
    IMG_FALSE = new QLabel();
    GPS_FALSE = new QLabel();
    IMU_FALSE = new QLabel();

    IMG_MISS->setText(QString::fromStdString("M:" + con->intToStr(img_miss_num)));
    GPS_MISS->setText(QString::fromStdString("M:" + con->intToStr(gps_miss_num)));
    IMU_MISS->setText(QString::fromStdString("M:" + con->intToStr(imu_miss_num)));
    IMG_FALSE->setText(QString::fromStdString("F:" + con->intToStr(img_false_num)));
    GPS_FALSE->setText(QString::fromStdString("F:" + con->intToStr(gps_false_num)));
    IMU_FALSE->setText(QString::fromStdString("F:" + con->intToStr(imu_false_num)));

    GPS_ImageLabel = new QLabel();
    IMU_ImageLabel = new QLabel();
    Camera_ImageLabel = new QLabel();

    GPS_ImageLabel->setScaledContents(true);
    IMU_ImageLabel->setScaledContents(true);
    Camera_ImageLabel->setScaledContents(true);

    std::cout << "icon dir = " << LocContext::getInstance()->getIconDir() << std::endl;
    red_pixmap.load((LocContext::getInstance()->getIconDir() + "/r.png").c_str());
    green_pixmap.load((LocContext::getInstance()->getIconDir() + "/g.png").c_str());

    GPS_ImageLabel->setMaximumHeight(30);
    GPS_ImageLabel->setMaximumWidth(30);
    GPS_ImageLabel->setPixmap(red_pixmap);

    IMU_ImageLabel->setMaximumHeight(30);
    IMU_ImageLabel->setMaximumWidth(30);
    IMU_ImageLabel->setPixmap(red_pixmap);

    Camera_ImageLabel->setMaximumHeight(30);
    Camera_ImageLabel->setMaximumWidth(30);
    Camera_ImageLabel->setPixmap(red_pixmap);

    pixmap_width = 20;
    pixmap_hight = 20;
    //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));

    QLabel *DGPS_state = new QLabel(QWidget::tr("DGPS_state:"));
    if (show_dgps_info)
    {
        DGPS_ImageLabel = new QLabel();
        DGPS_ImageLabel->setScaledContents(true);
        DGPS_ImageLabel->setMaximumHeight(30);
        DGPS_ImageLabel->setMaximumWidth(30);
        DGPS_ImageLabel->setPixmap(red_pixmap);

        //DGPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    }

    Button_run_loc = new QPushButton;
    Button_run_loc->setText("run_loc");
    Button_run_loc->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/loc.png").c_str()));
    connect(Button_run_loc, SIGNAL(clicked()), this, SLOT(Button_run_loc_event()));

    QPushButton *Button_ReInit = new QPushButton;
    Button_ReInit->setText("ReInit");
    connect(Button_ReInit, SIGNAL(clicked()), this, SLOT(Button_ReInit_event()));

    QPushButton *Button_QuickInit = new QPushButton;
    Button_QuickInit->setText("QuickInit");
    connect(Button_QuickInit, SIGNAL(clicked()), this, SLOT(Button_QuickInit_event()));

    Button_Reset = new QPushButton;
    Button_Reset->setText("Reset");
    Button_Reset->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/restart.png").c_str()));
    connect(Button_Reset, SIGNAL(clicked()), this, SLOT(Button_Reset_event()));

    Button_Start = new QPushButton;
    Button_Start->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Start, SIGNAL(clicked()), this, SLOT(Button_Start_event()));

    Button_Stop = new QPushButton;
    Button_Stop->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Stop, SIGNAL(clicked()), this, SLOT(Button_Stop_event()));

#ifndef DEV_BUILD
    QPushButton *Button_StartSrv = new QPushButton;
    Button_StartSrv->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_StartSrv, SIGNAL(clicked()), this, SLOT(Button_StartSrv_event()));

    QPushButton *Button_StopSrv = new QPushButton;
    Button_StopSrv->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_StopSrv, SIGNAL(clicked()), this, SLOT(Button_StopSrv_event()));

    Button_RestartSrv = new QPushButton;
    Button_RestartSrv->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/restart.png").c_str()));
    connect(Button_RestartSrv, SIGNAL(clicked()), this, SLOT(Button_RestartSrv_event()));

    Button_QuerySrv = new QPushButton;
    Button_QuerySrv->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/query.png").c_str()));
    connect(Button_QuerySrv, SIGNAL(clicked()), this, SLOT(Button_QuerySrv_event()));

    ComboBox_Rosbag = new QComboBox();
    ComboBox_Rosbag->addItem(QWidget::tr("Choose Rosbag"));
    connect(ComboBox_Rosbag, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Rosbag_valueChanged()));

    for (int i = 0; i < con->rosbag_list.size(); i++) {
        // std::cout<<con->rosbag_list[i]<<std::endl;
        if (i == 0) {
            cur_rosbag = con->rosbag_list[i].c_str();
        }
        ComboBox_Rosbag->addItem(QWidget::tr(con->fileName(con->rosbag_list[i]).c_str()));
    }

    Button_Play_Start = new QPushButton;
    Button_Play_Start->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Play_Start, SIGNAL(clicked()), this, SLOT(Button_Play_Start_event()));

    Button_Play_Stop = new QPushButton;
    Button_Play_Stop->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Play_Stop, SIGNAL(clicked()), this, SLOT(Button_Play_Stop_event()));

    QLabel *delay_info = new QLabel(QWidget::tr("Delay:"));
    delay_result = new QLabel;
    delay_result->setText("0/0");
    QHBoxLayout *dLayout = new QHBoxLayout;
    dLayout->addWidget(delay_info);
    dLayout->addWidget(delay_result);

    QLabel *status_info = new QLabel(QWidget::tr("Status:"));
    status_result = new QLabel;
    status_result->setText("No Data");
    QHBoxLayout *rLayout = new QHBoxLayout;
    rLayout->addWidget(status_info);
    rLayout->addWidget(status_result);

    error_msg_sub = nh_.subscribe<std_msgs::String>("add_error", 100, &TeleopPanel::on_add_error, this);
    log_status_sub = nh_.subscribe<std_msgs::String>("add_status", 100, &TeleopPanel::on_add_status, this);
    delay_sub = nh_.subscribe<std_msgs::String>("add_delay", 100, &TeleopPanel::on_add_delay, this);

    Button_SwitchMode = new QPushButton;
    Button_SwitchMode->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/expand.png").c_str()));
    connect(Button_SwitchMode, SIGNAL(clicked()), this, SLOT(Button_SwitchMode_event()));

    Button_Config = new QPushButton;
    Button_Config->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/gear.png").c_str()));
    connect(Button_Config, SIGNAL(clicked()), this, SLOT(Button_Open_config()));

    Button_testSSH = new QPushButton;
    Button_testSSH->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/ssh.png").c_str()));
    connect(Button_testSSH, SIGNAL(clicked()), this, SLOT(Button_Test_ssh()));
#endif

    ComboBox_View = new QComboBox();
    ComboBox_View->addItem(QWidget::tr("Choose View Mode"));
    ComboBox_View->addItem(QWidget::tr("Vehicle_View_FPS"));
    ComboBox_View->addItem(QWidget::tr("Vehicle_View_Follow"));
    ComboBox_View->addItem(QWidget::tr("Top-Down_2D_View"));
    ComboBox_View->addItem(QWidget::tr("Free_3D_View"));
    ComboBox_View->addItem(QWidget::tr("Debug_View"));
    connect(ComboBox_View, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_View_valueChanged()));

    ComboBox_Database = new QComboBox;
    ComboBox_Database->addItem(QWidget::tr("Choose Database"));
    connect(ComboBox_Database, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Database_valueChanged()));

    for (int i = 0; i < con->database_list.size(); i++) {
        // std::cout<<con->database_list[i]<<std::endl;
        if (i == 0) {
            cur_database = con->database_list[i].c_str();
        }
        ComboBox_Database->addItem(QWidget::tr(con->fileName(con->database_list[i]).c_str()));
    }

    ComboBox_Database->addItem(QWidget::tr("CC_Zone"));
    ComboBox_Database->addItem(QWidget::tr("MPG_Zone"));

    // Open paint
    /*Button_Paint = new QPushButton;
    Button_Paint->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/open.png").c_str()));
    connect(Button_Paint, SIGNAL(clicked()), this, SLOT(Button_Paint_event()));

    Button_Paint_Clear = new QPushButton;
    Button_Paint_Clear->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/clear.png").c_str()));
    connect(Button_Paint_Clear, SIGNAL(clicked()), this, SLOT(Button_Paint_Clear_event()));

    ComboBox_Paint = new QComboBox;
    ComboBox_Paint->addItem(QWidget::tr("Choose Paint"));
    connect(ComboBox_Paint, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Paint_valueChanged()));*/

    ComboBox_Execute = new QComboBox;
    ComboBox_Execute->addItem(QWidget::tr("Real Time"));
    ComboBox_Execute->addItem(QWidget::tr("Simulate rtv"));
    //    ComboBox_Execute->addItem(QWidget::tr("Dbgsnippet"));
    connect(ComboBox_Execute, SIGNAL(currentIndexChanged(int)), this, SLOT(ComboBox_Execute_valueChanged()));

    Debug_result = new QTextBrowser;
    Debug_result->setFontFamily("Consolas");
    Debug_result->setFontPointSize(9);
    Debug_result->setFontItalic(true);
    //Debug_result->setMaximumHeight(30);

    labelImage = new QLabel();
    // image showing reprojection err mean and std error
    labelImage_rprj_err = new QLabel;
    //labelLocImage = new QLabel;
    widgetLocImage = new ImageWidget;
    widgetLocImage->setMinimumHeight(400);
    widgetLocImage->setMinimumWidth(640);
    labelLocImage = widgetLocImage->labelLocImage;

    cv_image_width = 410;
    cv_image_hight = 240;
    cv_image.create(cv_image_hight, cv_image_width, CV_8UC3);
    cv_image_rprj_err.create(cv_image_hight, cv_image_width, CV_8UC3);
    for (int i = 0; i < cv_image_width; i++)
        for (int j = 0; j < cv_image_hight; j++)
            for (int k = 0; k < 3; k++)
                cv_image.at<cv::Vec3b>(j, i)[k] = 255;


    // grid
    cv::line(cv_image, cv::Point2d(28, 200), cv::Point2d(cv_image_width - 28, 200), colorBlack);
    cv::line(cv_image, cv::Point2d(30, 20), cv::Point2d(30, cv_image_hight - 37), colorBlack);
    cv::line(cv_image, cv::Point2d(cv_image_width - 30, 20), cv::Point2d(cv_image_width - 30, cv_image_hight - 37), colorBlack);
    for (int i = 0; i < 5; i++)
        cv::line(cv_image, cv::Point2d(28, 20 + i * 36), cv::Point2d(cv_image_width - 28, 20 + i * 36), colorGrey, 1);
    for (int i = 1; i < 10; i++)
        cv::line(cv_image, cv::Point2d(30 + i * 35, 18), cv::Point2d(30 + i * 35, 200), colorGrey, 1);

    // horizontal axis
    for (int i = 0; i < 6; i++) {
        if(i == 5)
            cv::putText(cv_image, con->intToStr(i * 20) + "(frames)", cv::Point2d(25 + i * 70 - 35, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        else
            cv::putText(cv_image, con->intToStr(i * 20), cv::Point2d(25 + i * 70, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
    }

    cv_image.copyTo(cv_image_rprj_err);

    // vertical axis
    for (int i = 0; i < 6; i++)
        cv::putText(cv_image, con->intToStr(i * 10), cv::Point2d(5, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
    for (int i = 0; i < 6; i++)
        cv::putText(cv_image, con->intToStr(i * 10), cv::Point2d(cv_image_width - 25, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);

    for (int i = 0; i < 6; i++)
        cv::putText(cv_image_rprj_err, con->intToStr(i * 2), cv::Point2d(5, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
    for (int i = 0; i < 6; i++)
        cv::putText(cv_image_rprj_err, con->intToStr(i * 2), cv::Point2d(cv_image_width - 25, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);

    std::stringstream text;
    text << "Uncertainty (cm)";
    cv::putText(cv_image, text.str(), cv::Point2d(30, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorRed, 1);
    text.str("");
    text << "";
    if (show_dgps_info)
    {
        text << "Error_with_DGPS (cm)";
        cv::putText(cv_image, text.str(), cv::Point2d(220, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorBlue, 1);
        text.str("");
        text << "";
    }
    QImage img = con->cvMat2QImage(cv_image);
    labelImage->setPixmap(QPixmap::fromImage(img).scaled(cv_image_width, cv_image_hight));
    labelImage->setScaledContents(true);

    text << "-Mean";
    cv::putText(cv_image_rprj_err, text.str(), cv::Point2d(30, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorGreen, 1);
    text.str("");
    text << "";
    text << "-Standard Deviation";
    cv::putText(cv_image_rprj_err, text.str(), cv::Point2d(100, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorPurple, 1);
    text.str("");
    text << "";
    QImage rprj_err_image = con->cvMat2QImage(cv_image_rprj_err);
    labelImage_rprj_err->setPixmap(QPixmap::fromImage(rprj_err_image).scaled(cv_image_width, cv_image_hight));
    labelImage_rprj_err->setScaledContents(true);

    QPushButton *Button_Start_all = new QPushButton;
    Button_Start_all->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/start.png").c_str()));
    connect(Button_Start_all, SIGNAL(clicked()), this, SLOT(Button_Start_all_event()));

    QPushButton *Button_Stop_all = new QPushButton;
    Button_Stop_all->setIcon(QPixmap((LocContext::getInstance()->getIconDir() + "/stop.png").c_str()));
    connect(Button_Stop_all, SIGNAL(clicked()), this, SLOT(Button_Stop_all_event()));

#ifdef DEV_BUILD
    gridLayout = new QGridLayout;

    gridLayout->setColumnStretch(0, 1);   //表示第0列的比例
    gridLayout->setColumnStretch(1, 0.3); //表示第1列的比例
    /*gridLayout->setColumnStretch(2, 0.7);//表示第0列的比例
    gridLayout->setColumnStretch(3, 0.7);//表示第1列的比例
    gridLayout->setColumnStretch(4, 0.7);//表示第0列的比例
    gridLayout->setColumnStretch(5, 0.7);//表示第1列的比例
    gridLayout->setColumnStretch(6, 1);//表示第0列的比例
    gridLayout->setColumnStretch(7, 1);//表示第1列的比例
    gridLayout->setColumnStretch(8, 1);//表示第0列的比例 */

    gridLayout->setMargin(8);  //表示控件与窗体的左右边距
    gridLayout->setSpacing(5); //设置组件的间隔为5像素
    gridLayout->setVerticalSpacing(5);

    gridLayout->addWidget(Show_loc_match_info, 0, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_Loc, 0, 1 + 2);
    gridLayout->addWidget(Show_upd_match_info, 1, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_Upd, 1, 1 + 2);

    gridLayout->addWidget(Show_links_info, 2, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_Links, 2, 1 + 2);
    gridLayout->addWidget(Show_his_uncety_info, 3, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_His_uncety, 3, 1 + 2);

    gridLayout->addWidget(Show_frameId_info, 4, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_FrameId, 4, 1 + 2);

    gridLayout->addWidget(Show_rprj_err_info, 5, 0, 1, 3, Qt::AlignRight);
    gridLayout->addWidget(CheckBox_rprj_err, 5, 1 + 2);

    gridLayout->addWidget(Camera_state, 6, 0, Qt::AlignRight);
    gridLayout->addWidget(Camera_ImageLabel, 6, 1);
    gridLayout->addWidget(IMG_MISS, 6, 2);
    gridLayout->addWidget(IMG_FALSE, 6, 3);
    if (show_dgps_info)
    {
        gridLayout->addWidget(DGPS_state, 7, 0, Qt::AlignRight);
        gridLayout->addWidget(DGPS_ImageLabel, 7, 1);

        gridLayout->addWidget(GPS_state, 7, 2, Qt::AlignRight);
        gridLayout->addWidget(GPS_ImageLabel, 7, 3);
        gridLayout->addWidget(GPS_MISS, 7, 4);
        gridLayout->addWidget(GPS_FALSE, 7, 5);
    }
    else
    {
        gridLayout->addWidget(GPS_state, 7, 0, Qt::AlignRight);
        gridLayout->addWidget(GPS_ImageLabel, 7, 1);
        gridLayout->addWidget(GPS_MISS, 7, 2);
        gridLayout->addWidget(GPS_FALSE, 7, 3);
    }

    gridLayout->addWidget(IMU_state, 8, 0, Qt::AlignRight);
    gridLayout->addWidget(IMU_ImageLabel, 8, 1);
    gridLayout->addWidget(IMU_MISS, 8, 2);
    gridLayout->addWidget(IMU_FALSE, 8, 3);

    gridLayout->addWidget(Button_Start_all, 9, 0, 1, 2);
    gridLayout->addWidget(Button_Stop_all, 9, 2, 1, 2);


    stackedWidget = new QStackedWidget;
    stackedWidget->addWidget(labelImage);
    stackedWidget->addWidget(labelImage_rprj_err);
    stackedWidget->setCurrentIndex(0);
    gridLayout->addWidget(stackedWidget, 0, 2 + 2, 10, 8);
    if (show_debug_info)
        gridLayout->addWidget(Debug_result, 10, 0, 3, 6);

    gridLayout->addWidget(ComboBox_Database, 10, 6, 1, 4);
    gridLayout->addWidget(Button_Reset, 10, 10 , 1, 2);
    gridLayout->addWidget(ComboBox_Execute, 11, 6, 1, 4);
    gridLayout->addWidget(Button_Start, 11, 10);
    gridLayout->addWidget(Button_Stop, 11, 11);
    gridLayout->addWidget(ComboBox_View, 12, 6, 1, 3);
    gridLayout->addWidget(Button_run_loc, 12, 9, 1, 1);
    gridLayout->addWidget(Button_ReInit, 12, 10, 1, 1);
    gridLayout->addWidget(Button_QuickInit, 12, 11, 1, 1);

    setLayout(gridLayout);
#else

    QGroupBox *groupBoxVis = new QGroupBox(tr("Visual Setting"));
    QVBoxLayout *switchLayout = new QVBoxLayout;
    CheckBox_Loc->setText("loc_match ");
    CheckBox_Upd->setText("upd_match ");
    CheckBox_Links->setText("map_links ");
    CheckBox_His_uncety->setText("his_uncety");
    CheckBox_FrameId->setText("frm_info ");
    CheckBox_rprj_err->setText("rprj_err ");

    switchLayout->addWidget(CheckBox_Loc);
    switchLayout->addWidget(CheckBox_Upd);
    switchLayout->addWidget(CheckBox_Links);
    switchLayout->addWidget(CheckBox_His_uncety);
    switchLayout->addWidget(CheckBox_FrameId);
    switchLayout->addWidget(CheckBox_rprj_err);
    //switchLayout->addStretch();
    //QHBoxLayout *bh = new QHBoxLayout;
    //bh->addWidget(Button_Config);
    //bh->addWidget(Button_testSSH);
    //switchLayout->addLayout(bh);
    groupBoxVis->setLayout(switchLayout);
    groupBoxVis->setFlat(true);
    addFrame(groupBoxVis);

//    QGroupBox *groupBoxTools = new QGroupBox(tr("ToolBox"));
//    QVBoxLayout *switchLayout = new QVBoxLayout;
//    CheckBox_Loc->setText("loc_match ");
//    CheckBox_Upd->setText("upd_match ");
//    CheckBox_Links->setText("map_links ");
//    CheckBox_His_uncety->setText("his_uncety");
//    CheckBox_FrameId->setText("frm_info ");
//    CheckBox_rprj_err->setText("rprj_err ");
//
//    switchLayout->addWidget(CheckBox_Loc);
//    switchLayout->addWidget(CheckBox_Upd);
//    switchLayout->addWidget(CheckBox_Links);
//    switchLayout->addWidget(CheckBox_His_uncety);
//    switchLayout->addWidget(CheckBox_FrameId);
//    switchLayout->addWidget(CheckBox_rprj_err);
//    QHBoxLayout *bh = new QHBoxLayout;
//    bh->addWidget(Button_Config);
//    bh->addWidget(Button_testSSH);
//    switchLayout->addLayout(bh);
//    groupBoxVis->setLayout(switchLayout);
//    addFrame(groupBoxVis);

    QGroupBox *groupBoxDataState = new QGroupBox(tr("Data Status"));
    QHBoxLayout *h1 = new QHBoxLayout;
    h1->addWidget(Camera_state);
    h1->addWidget(Camera_ImageLabel);
    h1->addWidget(IMG_MISS);
    h1->addWidget(IMG_FALSE);

    QHBoxLayout *h2 = new QHBoxLayout;
    if (show_dgps_info)
    {
        h2->addWidget(DGPS_state);
        h2->addWidget(DGPS_ImageLabel);
        h2->addWidget(GPS_state);
        h2->addWidget(GPS_ImageLabel);
        h2->addWidget(GPS_MISS);
        h2->addWidget(GPS_FALSE);
    }
    else
    {

        h2->addWidget(GPS_state);
        h2->addWidget(GPS_ImageLabel);
        h2->addWidget(GPS_MISS);
        h2->addWidget(GPS_FALSE);
    }

    QHBoxLayout *h3 = new QHBoxLayout;
    h3->addWidget(IMU_state);
    h3->addWidget(IMU_ImageLabel);
    h3->addWidget(IMU_MISS);
    h3->addWidget(IMU_FALSE);

    // state
    QVBoxLayout *stateLayout = new QVBoxLayout;

    stateLayout->addLayout(h1);
    stateLayout->addLayout(h2);
    stateLayout->addLayout(h3);
    stateLayout->addLayout(dLayout);
    stateLayout->addLayout(rLayout);

    groupBoxDataState->setLayout(stateLayout);
    addFrame(groupBoxDataState);

    QGroupBox *groupBoxAlgoControl = new QGroupBox(tr("Algorithm Control"));
    // db, algo control
    QHBoxLayout *h4 = new QHBoxLayout;
    h4->addWidget(ComboBox_Database);
    h4->addWidget(Button_StartSrv);
    h4->addWidget(Button_StopSrv);
    h4->addWidget(Button_RestartSrv);
    h4->addWidget(Button_run_loc);
    // h4->addWidget(Button_QuerySrv);

    // rosbag play,pause
    QHBoxLayout *h5 = new QHBoxLayout;
    h5->addWidget(ComboBox_Rosbag);
    h5->addWidget(Button_Play_Start);
    h5->addWidget(Button_Play_Stop);
    h5->addWidget(Button_ReInit);
    h5->addWidget(Button_QuickInit);

    QVBoxLayout *controlLayout = new QVBoxLayout;
    controlLayout->addLayout(h4);
    controlLayout->addLayout(h5);
    groupBoxAlgoControl->setLayout(controlLayout);
    groupBoxAlgoControl->setFlat(true);
    addFrame(groupBoxAlgoControl);

    // view layout
    QGroupBox *groupBoxViewControl = new QGroupBox(tr("View Control"));
    QVBoxLayout *viewLayout = new QVBoxLayout;
    viewLayout->addWidget(ComboBox_View);
    groupBoxViewControl->setLayout(viewLayout);
    groupBoxViewControl->setFlat(true);
    addFrame(groupBoxViewControl);

    QGroupBox *groupBoxGuiControl = new QGroupBox(tr("GUI Control"));
    QHBoxLayout *guiLayout = new QHBoxLayout;
    guiLayout->addWidget(Button_Reset);
    guiLayout->addWidget(Button_SwitchMode);
    groupBoxGuiControl->setLayout(guiLayout);
    groupBoxGuiControl->setFlat(true);
    addFrame(groupBoxGuiControl);

    QGroupBox *groupBoxChart = new QGroupBox(tr("Visual Chart"));
    stackedWidget = new QStackedWidget;
    stackedWidget->addWidget(labelImage);
    stackedWidget->addWidget(labelImage_rprj_err);
    stackedWidget->setCurrentIndex(0);
    QVBoxLayout *vLayout = new QVBoxLayout;
    vLayout->addWidget(stackedWidget);
    groupBoxChart->setLayout(vLayout);
    groupBoxChart->setFlat(true);
    addFrame(groupBoxChart);

    /*QGroupBox *groupBoxImage = new QGroupBox(tr("Image Display"));
    QVBoxLayout *ivLayout = new QVBoxLayout;
    ivLayout->addWidget(widgetLocImage);
    groupBoxImage->setLayout(ivLayout);
    addFrame(groupBoxImage);*/

    QHBoxLayout *h1Layout = new QHBoxLayout;
    h1Layout->addWidget(groupBoxVis);
    h1Layout->addWidget(groupBoxChart);

    QHBoxLayout *h2Layout = new QHBoxLayout;
    h2Layout->addWidget(groupBoxDataState);

    QVBoxLayout *v1Layout = new QVBoxLayout;
    v1Layout->addWidget(groupBoxAlgoControl);
    QHBoxLayout *hh1Layout = new QHBoxLayout;
    hh1Layout->addWidget(groupBoxViewControl);
    hh1Layout->addWidget(groupBoxGuiControl);
    v1Layout->addLayout(hh1Layout);

    h2Layout->addLayout(v1Layout);

    QVBoxLayout *fullVLayout = new QVBoxLayout;
    fullVLayout->addLayout(h1Layout);
    fullVLayout->addLayout(h2Layout);
    //fullVLayout->addWidget(widgetLocImage);

    //if(show_debug_info)
        //fullVLayout->addWidget(Debug_result);

    setLayout(fullVLayout);


    if("False" == LocContext::getInstance()->getParam("Control", "switch_remote_control"))
    {

        Button_StartSrv->setEnabled(false);
        Button_StopSrv->setEnabled(false);
        Button_RestartSrv->setEnabled(false);
        //Button_StartSrv->setEnabled(false);

        Button_Play_Start->setEnabled(false);
        Button_Play_Stop->setEnabled(false);
    }
#endif
    CheckBox_rprj_err->setCheckState(Qt::Unchecked);
    CheckBox_Upd->setCheckState(Qt::Checked);
    CheckBox_FrameId->setCheckState(Qt::Checked);

    rprjErrMin = 0;
    rprjErrMax = 10;
    InitView();
}


#ifndef DEV_BUILD
void TeleopPanel::Button_StartSrv_event()
{
    std::stringstream ss;
    if(prev_database.empty() || cur_database != prev_database) {
        // remove prev db and copy newer db to loc_database

        ss << "sudo rm -f " << LocContext::getInstance()->getParam("Control", "abox_db_root") << "* ; ";

        // copy to default db path /usr/local/ygomi/roadDB/algo_res/loc_database
        ss << "sudo cp -f " << LocContext::getInstance()->getParam("Control", "database_path") << cur_database << "/* "
           << LocContext::getInstance()->getParam("Control", "abox_db_root");
        con->do_cmd(ss.str(), true, true);

        prev_database = cur_database;
    }
    ss.str("");
    ss<<"";
    ss << "sudo service road_probe start | xargs | sed \"s,\\x1B\\[[0-9;]*[a-zA-Z],\\n,g\" ";
    std::vector<std::string> status_algo = con->do_cmd(ss.str(), true, false);
}
void TeleopPanel::Button_StopSrv_event()
{
    std::stringstream ss;
    // ss << "sudo service road_ros stop; sudo service road_probe stop";
    ss << "sudo service road_probe stop | xargs | sed \"s,\\x1B\\[[0-9;]*[a-zA-Z],\\n,g\" ";
    std::vector<std::string> status_algo = con->do_cmd(ss.str(), true, false);
}

void TeleopPanel::Button_RestartSrv_event()
{
    std::stringstream ss;
    if(prev_database.empty() || cur_database != prev_database) {
        // remove prev db and copy newer db to loc_database

        ss << "sudo rm -f " << LocContext::getInstance()->getParam("Control", "abox_db_root") << "* ; ";

        // copy to default db path /usr/local/ygomi/roadDB/algo_res/loc_database
        ss << "sudo cp -f " << LocContext::getInstance()->getParam("Control", "database_path") << cur_database << "/* "
           << LocContext::getInstance()->getParam("Control", "abox_db_root");
        con->do_cmd(ss.str(), true, true);

        prev_database = cur_database;
    }
    ss.str("");
    ss<<"";
    ss << "sudo service road_probe restart | xargs | sed \"s,\\x1B\\[[0-9;]*[a-zA-Z],\\n,g\" ";
    std::vector<std::string> status_algo = con->do_cmd(ss.str(), true, false);
}

void TeleopPanel::Button_QuerySrv_event()
{
    std::stringstream ss;
    ss << "sudo service road_ros status | xargs | sed \"s,\\x1B\\[[0-9;]*[a-zA-Z],\\n,g\" ";
    std::vector<std::string> status_ros = con->do_cmd(ss.str(), true, true);
    ss.str("");
    ss << "";
    ss << "sudo service road_probe status | xargs | sed \"s,\\x1B\\[[0-9;]*[a-zA-Z],\\n,g\" ";
    std::vector<std::string> status_algo = con->do_cmd(ss.str(), true, true);

    if(status_ros.size() > 0 && status_algo.size() > 0) {
        std::string toolTip;
        toolTip += status_ros[0];
        toolTip += "\n";
        toolTip += status_algo[0];

        Button_QuerySrv->setToolTip(tr(toolTip.c_str()));
        Debug_result->setText(toolTip.c_str());
    }
}

void TeleopPanel::ComboBox_Rosbag_valueChanged()
{
    if(ComboBox_Rosbag->currentIndex() == 0) {
        // std::cout<<"Choose rosbag..."<<std::endl;
        return;
    }

    cur_rosbag = con->rosbag_list[ComboBox_Rosbag->currentIndex() - 1];
    std::cout<<"choose rosbag: " << cur_rosbag << std::endl;
    Debug_result->setText(ComboBox_Rosbag->currentText());
}
void TeleopPanel::Button_Play_Start_event()
{
    Button_Play_Stop_event();
    std::stringstream ss;
    // restart road_probe when run new bag
    ss << "sudo service road_probe restart ; ";
    ss << ssh_ros_env << "rosbag play" << " " << cur_rosbag;
    con->do_cmd(ss.str(), true, false);
}
void TeleopPanel::Button_Play_Stop_event()
{
    std::stringstream ss;
    ss << ssh_ros_env << "rosnode list";
    std::vector<std::string> re_lines = con->do_cmd(ss.str(), true, true);
    for (int i = 0; i < re_lines.size(); i++)
    {
        if (re_lines[i][1] == 'p' && re_lines[i][2] == 'l' && re_lines[i][3] == 'a' &&
            re_lines[i][4] == 'y' && re_lines[i][5] == '_')
        {
            std::stringstream ss;
            ss << ssh_ros_env << "rosnode kill " << re_lines[i] << " ; ";
            ss << "sudo service road_probe stop ; ";
            re_lines = con->do_cmd(ss.str(), true, true);
        }
    }
}
void TeleopPanel::on_add_error(const std_msgs::String::ConstPtr &msg)
{
    Debug_result->setText(msg->data.c_str());
}

void TeleopPanel::on_add_delay(const std_msgs::String::ConstPtr &msg)
{
    delay_result->setText(msg->data.c_str());
}

void TeleopPanel::on_add_status(const std_msgs::String::ConstPtr &msg)
{
    status_result->setText(msg->data.c_str());
}
void TeleopPanel::Button_SwitchMode_event()
{

}
void TeleopPanel::Button_Open_config()
{

}
void TeleopPanel::Button_Test_ssh()
{
    con->testSSH();
}
#endif
void TeleopPanel::InitView()
{
    ComboBox_View->setCurrentIndex(4);
}
void TeleopPanel::sensor_check()
{
    cur_time = ros::Time::now();
    if ((cur_time - img_time).toSec() > 1)
    {
        //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(red_pixmap);
    }
    if ((cur_time - imu_time).toSec() > 1)
    {
        //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        IMU_ImageLabel->setPixmap(red_pixmap);
    }
    if ((cur_time - gps_time).toSec() > 2)
    {
        //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(red_pixmap);
    }
}
void TeleopPanel::on_sensor_status(const std_msgs::String::ConstPtr &msg)
{
    Debug_result->setText(msg->data.c_str());
    if (msg->data.find("img:0") != std::string::npos)
    {
    }
    if (msg->data.find("img:1") != std::string::npos)
    {
        //Camera_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(green_pixmap);
        img_time = ros::Time::now();
    }
    if (msg->data.find("img:2") != std::string::npos)
    {
        //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(red_pixmap);
        img_miss_num++;
    }
    if (msg->data.find("img:3") != std::string::npos)
    {
        //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        Camera_ImageLabel->setPixmap(red_pixmap);
        img_false_num++;
    }

    if (msg->data.find("gps:0") != std::string::npos)
    {
    }
    if (msg->data.find("gps:1") != std::string::npos)
    {
        //GPS_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(green_pixmap);
        gps_time = ros::Time::now();
    }
    if (msg->data.find("gps:2") != std::string::npos)
    {
        //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(red_pixmap);
        gps_miss_num++;
    }
    if (msg->data.find("gps:3") != std::string::npos)
    {
        //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        GPS_ImageLabel->setPixmap(red_pixmap);
        gps_false_num++;
    }

    if (msg->data.find("imu:0") != std::string::npos)
    {
    }
    if (msg->data.find("imu:1") != std::string::npos)
    {
        //IMU_ImageLabel->setPixmap(green_pixmap.scaled(pixmap_width, pixmap_hight));
        IMU_ImageLabel->setPixmap(green_pixmap);
        imu_time = ros::Time::now();
    }
    if (msg->data.find("imu:2") != std::string::npos)
    {
        //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        IMU_ImageLabel->setPixmap(red_pixmap);
        imu_miss_num++;
    }
    if (msg->data.find("imu:3") != std::string::npos)
    {
        //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
        IMU_ImageLabel->setPixmap(red_pixmap);
        imu_false_num++;
    }

    IMG_MISS->setText(QString::fromStdString("M:" + con->intToStr(img_miss_num)));
    GPS_MISS->setText(QString::fromStdString("M:" + con->intToStr(gps_miss_num)));
    IMU_MISS->setText(QString::fromStdString("M:" + con->intToStr(imu_miss_num)));
    IMG_FALSE->setText(QString::fromStdString("F:" + con->intToStr(img_false_num)));
    GPS_FALSE->setText(QString::fromStdString("F:" + con->intToStr(gps_false_num)));
    IMU_FALSE->setText(QString::fromStdString("F:" + con->intToStr(imu_false_num)));
}
void TeleopPanel::on_show_rprj_err(const loc_server::reproj_err_msg::ConstPtr &msg)
{
    int cur_frameId = msg->frame_id;
    if (msg->frame_id < start_frameId)
    {
        start_frameId_flag = 0;
        current_rprjErr_mean.clear();
        current_rprjErr_stdd.clear();
    }
    if (start_frameId_flag == 0)
    {
        start_frameId = msg->frame_id;
        start_frameId_flag = 1;
    }
    unsigned int inlier_count = msg->us.size();
    cv::Mat ProjErrNorm(inlier_count, 1, CV_32F);
    float sum = 0;
    for (unsigned int i = 0; i < inlier_count; ++i)
    {
        ProjErrNorm.at<float>(i, 0) = sqrt(msg->us[i] * msg->us[i] + msg->vs[i] * msg->vs[i]);
        // Ignore extremely big value
        if (ProjErrNorm.at<float>(i, 0) > 30)
            ProjErrNorm.at<float>(i, 0) = 30;
        sum += ProjErrNorm.at<float>(i, 0);
        //std::cout<<ProjErrNorm.at<float>(i, 0)<<std::endl;
        //std::cout<<std::endl;
    }

    /*cv::Mat meanProjErr,stdProjErr;
    cv::meanStdDev(ProjErrNorm,meanProjErr,stdProjErr);
    float mean = meanProjErr.at<float>(0,0) * 100;
    float stdd = stdProjErr.at<float>(0,0) * 100;*/

    float mean = sum / inlier_count;

    float stdd = 0;
    for (unsigned int i = 0; i < inlier_count; ++i)
    {
        stdd += (mean - ProjErrNorm.at<float>(i, 0)) * (mean - ProjErrNorm.at<float>(i, 0));
    }

    stdd = sqrt(stdd / inlier_count);

    if (mean > rprjErrMax)
        rprjErrMax = mean + 1;
    if (stdd > rprjErrMax)
        rprjErrMax = stdd + 1;
    if (mean < rprjErrMin)
        rprjErrMin = mean + 1;
    if (stdd < rprjErrMin)
        rprjErrMin = stdd + 1;

    //std::cout<<"min:\t" << rprjErrMin << "\tmax:\t" << rprjErrMax << std::endl;

    //std::cout<<"inlier count:\t"<< inlier_count << "\tmean: " << mean << "\tstdd: " << stdd << std::endl;

    if (rprjErrMax > 30)
    {
        for (unsigned int i = 0; i < inlier_count; ++i)
        {
            std::cout << ProjErrNorm.at<float>(i, 0) << "\t";
        }
        std::cout << std::endl;

        std::cout << "min:\t" << rprjErrMin << "\tmax:\t" << rprjErrMax << std::endl;

        std::cout << "inlier count:\t" << inlier_count << "\tmean: " << mean << "\tstdd: " << stdd << std::endl;
        rprjErrMax = 30;
    }

    current_rprjErr_mean.push_back(mean);
    current_rprjErr_stdd.push_back(stdd);

    const cv::Scalar colorBlack(0, 0, 0);
    const cv::Scalar colorGrey(200, 200, 200);
    const cv::Scalar colorGreen(0, 255, 0);
    const cv::Scalar colorPurple(255, 0, 255);

    cv_image_rprj_err.create(cv_image_hight, cv_image_width, CV_8UC3);
    for (int i = 0; i < cv_image_width; i++)
        for (int j = 0; j < cv_image_hight; j++)
            for (int k = 0; k < 3; k++)
                cv_image_rprj_err.at<cv::Vec3b>(j, i)[k] = 255;
    //for (int i = 0; i < 6; i++)
    //cv::line(cv_image_rprj_err, cv::Point2d(28, 20 + i * 36), cv::Point2d(cv_image_width - 28, 20 + i * 36), colorBlack);
    //for (int i = 0; i < 11; i++)
    //cv::line(cv_image_rprj_err, cv::Point2d(30 + i * 35, 20), cv::Point2d(30 + i * 35, cv_image_hight - 37), colorBlack);
    cv::line(cv_image_rprj_err, cv::Point2d(28, 200), cv::Point2d(cv_image_width - 28, 200), colorBlack);
    cv::line(cv_image_rprj_err, cv::Point2d(30, 20), cv::Point2d(30, cv_image_hight - 37), colorBlack);
    cv::line(cv_image_rprj_err, cv::Point2d(cv_image_width - 30, 20), cv::Point2d(cv_image_width - 30, cv_image_hight - 37), colorBlack);
    for (int i = 0; i < 5; i++)
        cv::line(cv_image_rprj_err, cv::Point2d(28, 20 + i * 36), cv::Point2d(cv_image_width - 28, 20 + i * 36), colorGrey, 1);
    for (int i = 1; i < 10; i++)
        cv::line(cv_image_rprj_err, cv::Point2d(30 + i * 35, 18), cv::Point2d(30 + i * 35, 200), colorGrey, 1);

    for (int i = 0; i < 6; i++)
    {
        int yAxisLabel = rprjErrMin + (rprjErrMax - rprjErrMin) * i / 5.0;
        cv::putText(cv_image_rprj_err, con->intToStr(yAxisLabel),
                    cv::Point2d(5, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        cv::putText(cv_image_rprj_err, con->intToStr(yAxisLabel),
                    cv::Point2d(cv_image_width - 25, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    colorBlack, 1);
    }

    if (current_rprjErr_mean.size() < 100)
    {
        for (int i = 0; i < 6; i++) {
            if(i == 5)
                cv::putText(cv_image_rprj_err, con->intToStr(start_frameId + i * 20) + "(frames)",
                        cv::Point2d(25 + i * 70 - 35, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
            else
                cv::putText(cv_image_rprj_err, con->intToStr(start_frameId + i * 20),
                        cv::Point2d(25 + i * 70, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        }

        for (int m = 0; m < current_rprjErr_mean.size(); m++)
        {
            float current_mean = current_rprjErr_mean[m];
            float current_stdd = current_rprjErr_stdd[m];
            //std::cout << "mean value: " << current_mean << "\tstdd value: " << current_stdd << std::endl;

            /*float meanyCoords = 200 - (200.0 * (current_mean - rprjErrMin) / (rprjErrMax - rprjErrMin));
            float stddyCoords = 200 - (200.0 * (current_stdd - rprjErrMin) / (rprjErrMax - rprjErrMin));*/
            float meanyCoords = 200 - ((current_mean - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
            float stddyCoords = 200 - ((current_stdd - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
            //std::cout<<"Debug:\t"<<meanyCoords<<", "<<stddyCoords<<std::endl;
            if (meanyCoords > 200 || stddyCoords > 200)
            {
                meanyCoords = 200;
                stddyCoords = 200;
            }
            if (meanyCoords < 20 || stddyCoords < 20)
            {
                meanyCoords = 20;
                stddyCoords = 20;
            }
            cv::circle(cv_image_rprj_err, cv::Point2d(30 + m * 3.5, meanyCoords), 1, colorGreen, 1);
            cv::circle(cv_image_rprj_err, cv::Point2d(30 + m * 3.5, stddyCoords), 1, colorPurple, 1);
            if (m > 0)
            {
                float current_mean_last = current_rprjErr_mean[m - 1];
                float current_stdd_last = current_rprjErr_stdd[m - 1];

                float meanyCoords_last = 200 - ((current_mean_last - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
                float stddyCoords_last = 200 - ((current_stdd_last - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
                if (meanyCoords_last > 200 || stddyCoords_last > 200)
                {
                    meanyCoords_last = 200;
                    stddyCoords_last = 200;
                }
                if (meanyCoords_last < 20 || stddyCoords_last < 20)
                {
                    meanyCoords_last = 20;
                    stddyCoords_last = 20;
                }
                cv::line(cv_image_rprj_err, cv::Point2d(m * 3.5 + 30, meanyCoords),
                         cv::Point2d((m - 1) * 3.5 + 30, meanyCoords_last), colorGreen, 1);

                cv::line(cv_image_rprj_err, cv::Point2d(m * 3.5 + 30, stddyCoords),
                         cv::Point2d((m - 1) * 3.5 + 30, stddyCoords_last), colorPurple, 1);
            }
        }
    }
    else
    {
        for (int i = 0; i < 6; i++) {
            if(i == 5)
                cv::putText(cv_image_rprj_err, con->intToStr(start_frameId + i * (cur_frameId - start_frameId) / 5) + "(frames)",
                        cv::Point2d(15 + i * 70 - 35, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
            else
                cv::putText(cv_image_rprj_err, con->intToStr(start_frameId + i * (cur_frameId - start_frameId) / 5),
                        cv::Point2d(15 + i * 70, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        }

        for (int m = 0; m < current_rprjErr_mean.size(); m++)
        {
            float current_mean = current_rprjErr_mean[m];
            float current_stdd = current_rprjErr_stdd[m];
            //std::cout << "mean value: " << current_mean << "\tstdd value: " << current_stdd << std::endl;

            float meanyCoords = 200 - ((current_mean - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
            float stddyCoords = 200 - ((current_stdd - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
            //std::cout<<"Debug:\t"<<meanyCoords<<", "<<stddyCoords<<std::endl;
            if (meanyCoords > 200 || stddyCoords > 200)
            {
                meanyCoords = 200;
                stddyCoords = 200;
            }
            if (meanyCoords < 20 || stddyCoords < 20)
            {
                meanyCoords = 20;
                stddyCoords = 20;
            }
            cv::circle(cv_image_rprj_err, cv::Point2d(30 + m * 350 / current_rprjErr_mean.size(), meanyCoords), 1, colorGreen, 1);
            cv::circle(cv_image_rprj_err, cv::Point2d(30 + m * 350 / current_rprjErr_mean.size(), stddyCoords), 1, colorPurple, 1);
            if (m > 0)
            {
                float current_mean_last = current_rprjErr_mean[m - 1];
                float current_stdd_last = current_rprjErr_stdd[m - 1];

                float meanyCoords_last = 200 - ((current_mean_last - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
                float stddyCoords_last = 200 - ((current_stdd_last - rprjErrMin) * (200 / (rprjErrMax - rprjErrMin)));
                if (meanyCoords_last > 200 || stddyCoords_last > 200)
                {
                    meanyCoords_last = 200;
                    stddyCoords_last = 200;
                }
                if (meanyCoords_last < 20 || stddyCoords_last < 20)
                {
                    meanyCoords_last = 20;
                    stddyCoords_last = 20;
                }
                cv::line(cv_image_rprj_err, cv::Point2d(30 + m * 350 / current_rprjErr_mean.size(), meanyCoords),
                         cv::Point2d(30 + (m - 1) * 350 / current_rprjErr_mean.size(), meanyCoords_last), colorGreen, 1);
                cv::line(cv_image_rprj_err, cv::Point2d(30 + m * 350 / current_rprjErr_mean.size(), stddyCoords),
                         cv::Point2d(30 + (m - 1) * 350 / current_rprjErr_mean.size(), stddyCoords_last), colorPurple, 1);
            }
        }
    }

    std::stringstream text;
    text << "-Mean";
    cv::putText(cv_image_rprj_err, text.str(), cv::Point2d(30, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorGreen, 1);
    text.str("");
    text << "";
    text << "-Standard Deviation";
    cv::putText(cv_image_rprj_err, text.str(), cv::Point2d(100, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorPurple, 1);
    text.str("");
    text << "";

    QImage img_rprj_err = con->cvMat2QImage(cv_image_rprj_err);
    labelImage_rprj_err->setPixmap(QPixmap::fromImage(img_rprj_err).scaled(cv_image_width, cv_image_hight));
    labelImage_rprj_err->setScaledContents(true);
}
void TeleopPanel::on_show_current_uncertainty(const loc_server::loc_predict_msg::ConstPtr &msg)
{

    int cur_frameId = msg->frame_id;
    if (msg->frame_id < start_frameId)
    {
        start_frameId_flag = 0;
        current_uncertainty.clear();
    }
    if (start_frameId_flag == 0)
    {
        start_frameId = msg->frame_id;
        start_frameId_flag = 1;
    }
    float cov_x = sqrt(msg->cov.x);
    float cov_y = sqrt(msg->cov.y);
    float cov_z = sqrt(msg->cov.z);
    float cur_cov = sqrt(cov_x * cov_x + cov_y * cov_y + cov_z * cov_z) * 100;
    //std::cout<<cur_cov<<std::endl;
    current_uncertainty.push_back(cur_cov);

    //     Eigen::Vector3f cur_posi = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //     current_position[cur_frameId] = cur_posi;

    const cv::Scalar colorRed(0, 0, 255);
    const cv::Scalar colorGrey(200, 200, 200);
    const cv::Scalar colorBlack(0, 0, 0);
    const cv::Scalar colorBlue(255, 0, 0);
    //cv_image.create(cv_image_hight, cv_image_width, CV_8UC3);

    for (int i = 0; i < cv_image_width; i++)
        for (int j = 0; j < cv_image_hight; j++)
            for (int k = 0; k < 3; k++)
                cv_image.at<cv::Vec3b>(j, i)[k] = 255;
    //for (int i = 0; i < 6; i++)
    //cv::line(cv_image, cv::Point2d(28, 20 + i * 36), cv::Point2d(cv_image_width - 28, 20 + i * 36), colorBlack);
    //for (int i = 0; i < 11; i++)
    //cv::line(cv_image, cv::Point2d(30 + i * 35, 20), cv::Point2d(30 + i * 35, cv_image_hight - 37), colorBlack);
    cv::line(cv_image, cv::Point2d(28, 200), cv::Point2d(cv_image_width - 28, 200), colorBlack);
    cv::line(cv_image, cv::Point2d(30, 20), cv::Point2d(30, cv_image_hight - 37), colorBlack);
    cv::line(cv_image, cv::Point2d(cv_image_width - 30, 20), cv::Point2d(cv_image_width - 30, cv_image_hight - 37), colorBlack);
    for (int i = 0; i < 5; i++)
        cv::line(cv_image, cv::Point2d(28, 20 + i * 36), cv::Point2d(cv_image_width - 28, 20 + i * 36), colorGrey, 1);
    for (int i = 1; i < 10; i++)
        cv::line(cv_image, cv::Point2d(30 + i * 35, 18), cv::Point2d(30 + i * 35, 200), colorGrey, 1);
    for (int i = 0; i < 6; i++)
        cv::putText(cv_image, con->intToStr(i * 10),
                    cv::Point2d(5, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
    for (int i = 0; i < 6; i++)
        cv::putText(cv_image, con->intToStr(i * 10),
                    cv::Point2d(cv_image_width - 25, cv_image_hight - (35 + i * 36)), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);

    if (current_uncertainty.size() < 100)
    {
        for (int i = 0; i < 6; i++) {
            if(i == 5)
                cv::putText(cv_image, con->intToStr(start_frameId + i * 20) + "(frames)",
                        cv::Point2d(25 + i * 70 - 35, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
            else
                cv::putText(cv_image, con->intToStr(start_frameId + i * 20),
                        cv::Point2d(25 + i * 70, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        }

        for (int m = 0; m < current_uncertainty.size(); m++)
        {
            float current_cov = current_uncertainty[m];
            if (current_cov > 180)
                current_cov = 180;
            cv::circle(cv_image, cv::Point2d(30 + m * 3.5, -current_cov * 1 + 200), 1, colorRed, 1);
            if (m > 0)
            {
                float current_cov_last = current_uncertainty[m - 1];
                if (current_cov_last > 180)
                    current_cov_last = 180;
                cv::line(cv_image, cv::Point2d(m * 3.5 + 30, -current_cov * 1 + 200),
                         cv::Point2d((m - 1) * 3.5 + 30, -current_cov_last * 1 + 200), colorRed, 1);
            }
        }
    }
    else
    {
        for (int i = 0; i < 6; i++) {
            if(i == 5)
                cv::putText(cv_image, con->intToStr(start_frameId + i * (cur_frameId - start_frameId) / 5) + "(frames)",
                        cv::Point2d(15 + i * 70 - 35, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
            else
                cv::putText(cv_image, con->intToStr(start_frameId + i * (cur_frameId - start_frameId) / 5),
                        cv::Point2d(15 + i * 70, cv_image_hight - 15), cv::FONT_HERSHEY_SIMPLEX, 0.35, colorBlack, 1);
        }

        for (int m = 0; m < current_uncertainty.size(); m++)
        {
            float current_cov = current_uncertainty[m];
            if (current_cov > 180)
                current_cov = 180;
            cv::circle(cv_image, cv::Point2d(30 + m * 350 / current_uncertainty.size(), -current_cov * 1 + 200), 1, colorRed, 1);
            if (m > 0)
            {
                float current_cov_last = current_uncertainty[m - 1];
                if (current_cov_last > 180)
                    current_cov_last = 180;
                cv::line(cv_image, cv::Point2d(30 + m * 350 / current_uncertainty.size(), -current_cov * 1 + 200),
                         cv::Point2d(30 + (m - 1) * 350 / current_uncertainty.size(), -current_cov_last * 1 + 200), colorRed, 1);
            }
        }
    }

    std::stringstream text;
    text << "1 Sigma Uncertainty (cm)";
    cv::putText(cv_image, text.str(), cv::Point2d(30, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorRed, 1);
    text.str("");
    text << "";
    if (show_dgps_info)
    {
        text << "Error_with_DGPS (cm)";
        cv::putText(cv_image, text.str(), cv::Point2d(220, 13), cv::FONT_HERSHEY_TRIPLEX, 0.4, colorBlue, 1);
        text.str("");
        text << "";
    }
    QImage img = con->cvMat2QImage(cv_image);
    labelImage->setPixmap(QPixmap::fromImage(img).scaled(cv_image_width, cv_image_hight));
    labelImage->setScaledContents(true);
}
void unixTime2Str(int64 time, std::string &time_string)
{
    time = time / 1000;
    std::cout << "time:" << time << std::endl;
    time_t t = (time_t)(time);
    struct tm *tm = localtime(&t);
    char date[20];
    strftime(date, sizeof(date), "%Y-%m-%d_%H-%M-%S", tm);
    std::cout << date << std::endl;
    time_string = std::string(date);
}
void TeleopPanel::on_new_run(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "new_run")
    {
        Button_Reset_event();
        flag_update_img = true;
    }
}
void TeleopPanel::on_img(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
      QImage img = con->cvMat2QImage(cv_ptr->image);

      labelLocImage->setPixmap(QPixmap::fromImage(img));
      labelLocImage->setScaledContents(true);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}
void TeleopPanel::on_db_ready_notice(const std_msgs::String::ConstPtr &msg)
{
    static bool isFirstTime = true;
    if(isFirstTime) {
        std::cout << "db ready" << std::endl;
        labelLocImage->setText("DB Ready");
        isFirstTime = false;
    }
}
void TeleopPanel::on_database(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    static bool isFirstTime = true;
    if(isFirstTime) {
        std::cout << "start loading DB" << std::endl;
        labelLocImage->setText("Loading DB");
        isFirstTime = false;
    }
    if (false == flag_update_img)
    {
        flag_update_img = true;
    }
}
void TeleopPanel::on_gps(const geometry_msgs::Point32::ConstPtr &msg)
{
    //
}
void TeleopPanel::on_update(const loc_server::loc_update_msg::ConstPtr &msg)
{
    flag_update_img = false;
    if (false == flag_sent_file_name)
    {
    #ifdef DEV_BUILD
        ComboBox_View->setCurrentIndex(4); // Free 3d
    #else
        ComboBox_View->setCurrentIndex(2); // 3rd Follow
    #endif
        std_msgs::String ss;
        ss.data = bag_name;
        publisher_file_name.publish(ss);
        flag_sent_file_name = true;
    }
}
void TeleopPanel::on_get_first_gps_time(const record_msg::gps_record::ConstPtr &gps_msg)
{
    if (true == button_start_all_flag)
    {
        double gps_timestamp = gps_msg->timestamp;
        unixTime2Str((int64)gps_timestamp, bag_name);
        if (!bag_name.empty())
        {
            button_start_all_flag = false;
            std::cout << "bag_name:" << bag_name << std::endl;
            Satrt_recording();
            Button_Start_event();
        }
    }
}
void TeleopPanel::Button_Start_all_event()
{
    Button_Stop_all_event();
    get_fist_gps_time_flag = false;
    std::string user_name = con->do_cmd("echo `whoami`", false, true)[0];
    std::string ubuntu_rosbag_folder = "/home/" + user_name + "/Documents/RosBag_Folder";
    std::cout << "ubuntu_rosbag_folder:" << ubuntu_rosbag_folder << std::endl;
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
    Record_dir = ubuntu_rosbag_folder + "/Defult_folder";
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
    button_start_all_flag = true;
}
void TeleopPanel::Satrt_recording()
{
    Stop_recording();
    std::stringstream ss;
    ss << "cd " << Record_dir << "&& rosbag record -O " << cur_database << "_" << bag_name << "_orignal_data"
       << " /gps_record "
       << "/img_record "
       << "/imu_pack";
    con->do_cmd(ss.str(), false, false);
    std::stringstream ss2;
    ss2 << "cd " << Record_dir << "&& rosbag record -O " << cur_database << "_" << bag_name << "_loc_server_data"
        << " /predict_server /update_server /match_server /gps_server /image_server /rviz_control /database_server /kfposition_server";
    con->do_cmd(ss2.str(), false, false);
}
void TeleopPanel::Stop_recording()
{
    for (int m = 0; m < 2; m++)
    {
        std::vector<std::string> re_lines = con->do_cmd("rosnode list", false, true);
        for (int i = 0; i < re_lines.size(); i++)
        {
            if (re_lines[i][1] == 'r' && re_lines[i][2] == 'e' && re_lines[i][3] == 'c' && re_lines[i][4] == 'o' && re_lines[i][5] == 'r' && re_lines[i][6] == 'd' && re_lines[i][7] == '_')
            {
                std::stringstream ss;
                ss << "rosnode kill " << re_lines[i];
                re_lines = con->do_cmd(ss.str(), false, true);
            }
        }
    }
}
void TeleopPanel::Button_Stop_all_event()
{
    Stop_recording();
    Button_Stop_event();
}
void TeleopPanel::Button_Start_event()
{
    Button_Stop_event();
    Button_Reset_event();
    Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));

    if (con->tryKillNode(cur_code))
    {
        Debug_result->setText("Restart...\n");
    }
    Debug_result->setText("Start...\n");

    if (ComboBox_Database->currentIndex() == 0)
    {
        Debug_result->setText("Please choose the database");
        return;
    }

    current_uncertainty.clear();
    current_rprjErr_stdd.clear();
    current_rprjErr_mean.clear();
    current_error.clear();
    current_position.clear();
    current_dgps.clear();
    start_frameId_flag = 0;

    std::stringstream ss;
    std::string set_ros_ip_str = remoteConHelper::getInst()->set_ros_ip_str;
    if("True" == LocContext::getInstance()->getParam("Control", "switch_remote_control"))
    {
        ss<<"export ROS_IP="<<LocContext::getInstance()->getParam("Control", "remote_ip")
           <<" && export ROS_MASTER_URI=http://"<<LocContext::getInstance()->getParam("Control", "remote_ip")<<":11311/ && ";
    }
    else
    {
        std::string user_name = con->do_cmd("echo `whoami`", false, true)[0];
        ss << "cd /home/" << user_name << "/ && ";
    }

    std::string code_path = LocContext::getInstance()->getParam("Control", "code_path");
    std::string database_path = LocContext::getInstance()->getParam("Control", "database_path");
    std::string bow_file = LocContext::getInstance()->getParam("Control", "bow_file");
    std::string rtv_path = LocContext::getInstance()->getParam("Control", "rtv_path");

    std::string cur_gps = "none", cur_imu = "none";
    if(cur_rtv.size() > 4) {
        std::string rtv_name = cur_rtv.substr(0, cur_rtv.size() - 4);
        cur_gps = rtv_name + ".gps";
        cur_imu = rtv_name + ".imu";
    }
    switch (ComboBox_Execute->currentIndex())
    {
    case 0: //loc_algo_ros
        cur_code = "/loc_algo_ros";
        cur_execute = "examples/LocRealTimeRosAlgo/LocRealTimeRosAlgo";
        cur_execute_to_kill = "LocRealTimeRosAlgo";
        ss << "mkdir -p 1-Algo_Log_Dir && cd 1-Algo_Log_Dir &&"
           << code_path << cur_execute                        << " "
           << " -c " << code_path << "bin/resources/CONTI/CONTI65.yaml" << " "
           << " -f " << code_path << "config/slamConfig_loc.json"           << " "
           << " -d " << database_path << cur_database                   << " "
           << " -s " << "0"                                                        << " "
           << " -r " << "100000"                                                   << " "
           << " -v " << bow_file                                       << " "
           << " -l " << code_path << "config/locConfigRosViewer.json"   << " "
           << " -i " << code_path << "config/IMUConfig_Conti65.json"    << " "
           << " -t " << "live" << " "
           << " -o " << "output"
           << " --ologpath ./log.txt";
        break;
    case 1: //loc_realtime
        cur_code = "/loc_realtime";
        cur_execute = "examples/LocRealTime/loc_realtime";
        cur_execute_to_kill = "loc_realtime";
        ss << code_path << cur_execute << " " << code_path << "bin/resources/CONTI/CONTI65.yaml"
           << " " << code_path << "config/slamConfig.json"
           << " " << database_path << cur_database << " " << rtv_path << cur_rtv << " " << cur_gps << " "
           << "0"
           << " "
           << "100000"
           << " " << bow_file<< " " << code_path << "config/locConfigRosViewer.json"
           << " " << code_path << "config/IMUConfig_Conti65.json"
           << " " << cur_imu;
        break;
    }
    con->do_cmd(ss.str(), true, false);
}
void TeleopPanel::Button_Stop_event()
{
    std_msgs::String ss;
    ss.data = "stopAlgo";
    publisher_stop_algo.publish(ss);
    Debug_result->setText("Stop running\n");

    std::cout << "current rosnode" << cur_code << ::endl;
    if (con->tryKillNode(cur_code, false, true))
    {
        Debug_result->setText("Stop running\n");
    }
    std::stringstream ss2;
    ss2 << "killall " << cur_execute_to_kill;
    con->do_cmd(ss2.str(), true, false);
}
void TeleopPanel::Button_Reset_event()
{
    current_uncertainty.clear();
    current_rprjErr_stdd.clear();
    current_rprjErr_mean.clear();
    current_error.clear();
    current_position.clear();
    current_dgps.clear();
    start_frameId_flag = 0;

    img_miss_num = 0;
    img_false_num = 0;
    gps_miss_num = 0;
    gps_false_num = 0;
    imu_miss_num = 0;
    imu_false_num = 0;

    IMG_MISS->setText(QString::fromStdString("M:" + con->intToStr(img_miss_num)));
    GPS_MISS->setText(QString::fromStdString("M:" + con->intToStr(gps_miss_num)));
    IMU_MISS->setText(QString::fromStdString("M:" + con->intToStr(imu_miss_num)));
    IMG_FALSE->setText(QString::fromStdString("F:" + con->intToStr(img_false_num)));
    GPS_FALSE->setText(QString::fromStdString("F:" + con->intToStr(gps_false_num)));
    IMU_FALSE->setText(QString::fromStdString("F:" + con->intToStr(imu_false_num)));

    //Camera_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //IMU_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    //GPS_ImageLabel->setPixmap(red_pixmap.scaled(pixmap_width, pixmap_hight));
    Camera_ImageLabel->setPixmap(red_pixmap);
    IMU_ImageLabel->setPixmap(red_pixmap);
    GPS_ImageLabel->setPixmap(red_pixmap);

    CheckBox_Links->setCheckState(Qt::Unchecked);
    CheckBox_His_uncety->setCheckState(Qt::Unchecked);
    // rviz_control_msg_.Views_type_choose = 3;
    // Debug_result->setText("Free_3Dview_");
    rviz_control_msg_.Links_switch = 2;
    rviz_control_msg_.His_uncety_switch = 2;
    rviz_control_msg_.Iamge_id_switch = 0;
    rviz_control_msg_.Loc_match_switch = 0;
    rviz_control_msg_.Upd_match_switch = 0;
    rviz_control_publisher_.publish(rviz_control_msg_);
}
void TeleopPanel::Button_run_loc_event()
{
    if(ros::ok())
    {
        std_msgs::String ss;
        ss.data = "runloc";
        publisher_run_loc.publish(ss);
    }
}
void TeleopPanel::Button_QuickInit_event() {
    if (ros::ok())
    {
        std_msgs::String ss;
        ss.data = "QuickInit";
        publisher_Quickinit_loc.publish(ss);

        current_uncertainty.clear();
        start_frameId_flag = 0;
    }
}

void TeleopPanel::Button_ReInit_event()
{
    if (ros::ok())
    {
        std_msgs::String ss;
        ss.data = "Reinit";
        publisher_Reinit_loc.publish(ss);

        current_uncertainty.clear();
        start_frameId_flag = 0;
    }
}
void TeleopPanel::ComboBox_Execute_valueChanged()
{
    if (ros::ok() && rviz_control_publisher_)
    {
        switch (ComboBox_Execute->currentIndex())
        {
        case 1:
            cur_execute = "examples/LocRealTimeRosAlgo/LocRealTimeRosAlgo";
            Debug_result->setText("cur_execute = examples/LocRealTimeRosAlgo/LocRealTimeRosAlgo");
            break;
        case 2:
            cur_execute = "examples/LocRealTime/loc_realtime";
            Debug_result->setText("cur_execute = examples/LocRealTime/loc_realtime");
            break;
        }
    }
}
void TeleopPanel::ComboBox_Database_valueChanged()
{
    ComboBox_Database_currentIndex = ComboBox_Database->currentIndex();
    if(ComboBox_Database_currentIndex == 0) {
        // std::cout<<"Choose databse..."<<std::endl;
        return;
    }

    QString text = ComboBox_Database->currentText();
    Debug_result->setText(text);
    if(text == "CC_Zone" || text == "MPG_Zone") {
        cur_database = text.toUtf8().constData();
    }
    else{
        if(con->database_list.size() <= 0)
            return;
        if(ComboBox_Database_currentIndex < 2)
            return;

        cur_database = con->database_list[ComboBox_Database_currentIndex-1];
        std::cout<<"choose database: " << cur_database << std::endl;
    }

    std::stringstream ss1;

    // load file referring b/t db and paint
    std::string res_root=LocContext::getInstance()->getPaintDir();
    std::string ref_file=res_root + "paint_ref.txt";
    std::ifstream ifs(ref_file.c_str(), std::ios::in);
    if(!ifs.is_open())
        std::cerr<<"error opening ref file: " << ref_file << std::endl;
    else
    {
        char buffer[1024];
        int line_count = 0;
        // db,semi_dense,paint
        while(!ifs.eof()) {
            ifs.getline(buffer, 256);
            ++line_count;
            if(buffer[0] == '#' || buffer[0] == ' ' || buffer[0] == '\t' || buffer[0] == '\n')
                continue;
            std::vector<std::string> files = LocContext::getInstance()->split(buffer, ',');
            if(files.size() != 3) {
                std::cout<<"illegal line: " << line_count << std::endl;
            }
            else
            {
                if(-1 != cur_database.find(files[0])) {
                    std::cout<<"find ref of " << cur_database << " semidense: " << files[1] << " paint: " << files[2] <<  std::endl;
                    ss1 << files[1] << "," << files[2] << ",none,none";
                    std_msgs::String paint_msg;
                    paint_msg.data = ss1.str();
                    paint_publisher_.publish(paint_msg);
                    break;
                }
            }
        }
    }
}
/*void TeleopPanel::Button_Paint_event()
{
    QString res_root(QString::fromStdString(LocContext::getInstance()->getPaintDir()));
    QDir res_dir(res_root);
    QStringList file_names = QFileDialog::getOpenFileNames(this, QWidget::tr("Choose Paint"), res_root, QWidget::tr("Paints (*.txt *.ply)"));

    std::cout << "Paint_dir:" << res_root.toUtf8().constData() << std::endl;

    for (int i = 0; i < file_names.size(); i++) {
        QString rel_path = res_dir.relativeFilePath(file_names.at(i));
        std::cout<<"rel_path: " <<rel_path.toUtf8().constData()<<std::endl;
        ComboBox_Paint->addItem(QWidget::tr(rel_path.toUtf8().constData()));
    }
}
void TeleopPanel::Button_Paint_Clear_event()
{
    std_msgs::String paint_msg;
    paint_msg.data = std::string("deleteAll");
    ComboBox_Paint->setCurrentIndex(0);
    Debug_result->setText("");
    paint_publisher_.publish(paint_msg);
}
void TeleopPanel::ComboBox_Paint_valueChanged()
{
    std::string res_root = LocContext::getInstance()->getPaintDir();
    QString file_name = ComboBox_Paint->currentText();
    std::cout<<"file_name: " << file_name.toUtf8().constData() << std::endl;
    if(ComboBox_Paint->currentIndex() != 0) {
        std_msgs::String paint_msg;
        paint_msg.data = res_root + file_name.toUtf8().constData();
        paint_publisher_.publish(paint_msg);

        std::cout<<"ComboBox_Paint_valueChanged"<<std::endl;
        Debug_result->setText(file_name);
    }
}*/
void TeleopPanel::CheckBox_rprj_err_onStateChanged(int state)
{
    if (state == Qt::Checked)
    {
        stackedWidget->setCurrentIndex(1);
    }
    if (state == Qt::Unchecked)
    {
        stackedWidget->setCurrentIndex(0);
    }
}
void TeleopPanel::ComboBox_View_valueChanged()
{
    if(ros::ok() && view_control_publisher_)
    {
        int currentIndex = ComboBox_View->currentIndex();
        QString currentText = ComboBox_View->currentText();
        switch (currentIndex) {
            case 1: //Vehicle_view_FPS
                view_control_msg_.Views_type_choose = 1;
                Debug_result->setText("Vehicle_view_FPS");
                break;
            case 2: //Vehicle_view_3rdFollow
                view_control_msg_.Views_type_choose = 6;
                Debug_result->setText("Vehicle_view_3rdFollow");
                break;
            case 3: //Top-down_2Dview
                view_control_msg_.Views_type_choose = 2;
                Debug_result->setText("Top-down_2Dview_");
                break;
            case 4: //Free_3Dview
                view_control_msg_.Views_type_choose = 3;
                Debug_result->setText("Free_3Dview_");
                break;
            case 5: //Debug
                view_control_msg_.Views_type_choose = 4;
                Debug_result->setText("Debug_");
                break;
            case 6:
                view_control_msg_.Views_type_choose = 5;
                Debug_result->setText("Simplest_");
                break;
            default:
                Debug_result->setText("default");
                break;
        }
        view_control_publisher_.publish(view_control_msg_);
    }
}
void TeleopPanel::CheckBox_His_uncety_onStateChanged(int state)
{
    if (ros::ok() && rviz_control_publisher_)
    {
        if (state == Qt::Checked)
            rviz_control_msg_.His_uncety_switch = 1;
        if (state == Qt::Unchecked)
            rviz_control_msg_.His_uncety_switch = 2;

        rviz_control_msg_.Iamge_id_switch = 0;
        rviz_control_msg_.Loc_match_switch = 0;
        rviz_control_msg_.Upd_match_switch = 0;
        rviz_control_msg_.Links_switch = 0;
        rviz_control_publisher_.publish(rviz_control_msg_);
        std::cout << "CheckBox_His_uncety_onStateChanged" << std::endl;
    }
}
void TeleopPanel::CheckBox_Links_onStateChanged(int state)
{
    if (ros::ok() && rviz_control_publisher_)
    {
        if (state == Qt::Checked)
            rviz_control_msg_.Links_switch = 1;
        if (state == Qt::Unchecked)
            rviz_control_msg_.Links_switch = 2;

        rviz_control_msg_.Iamge_id_switch = 0;
        rviz_control_msg_.Loc_match_switch = 0;
        rviz_control_msg_.Upd_match_switch = 0;
        rviz_control_publisher_.publish(rviz_control_msg_);
        std::cout << "CheckBox_Links_onStateChanged" << std::endl;
    }
}
void TeleopPanel::CheckBox_FrameId_onStateChanged(int state)
{
    if (ros::ok() && rviz_control_publisher_)
    {
        if (state == Qt::Checked)
            rviz_control_msg_.Iamge_id_switch = 1;
        if (state == Qt::Unchecked) // "选中"
            rviz_control_msg_.Iamge_id_switch = 2;

        rviz_control_msg_.Loc_match_switch = 0;
        rviz_control_msg_.Upd_match_switch = 0;
        rviz_control_msg_.Links_switch = 0;
        rviz_control_publisher_.publish(rviz_control_msg_);
        std::cout << "CheckBox_FrameId_onStateChanged" << std::endl;
    }
}
void TeleopPanel::CheckBox_Upd_onStateChanged(int state)
{
    if (ros::ok() && rviz_control_publisher_)
    {
        if (state == Qt::Checked)
            rviz_control_msg_.Upd_match_switch = 1;
        if (state == Qt::Unchecked) // "选中"
            rviz_control_msg_.Upd_match_switch = 2;

        rviz_control_msg_.Iamge_id_switch = 0;
        rviz_control_msg_.Loc_match_switch = 0;
        rviz_control_msg_.Links_switch = 0;
        rviz_control_publisher_.publish(rviz_control_msg_);
        std::cout << "CheckBox_Upd_onStateChanged" << std::endl;
    }
}
void TeleopPanel::CheckBox_Loc_onStateChanged(int state)
{
    if (ros::ok() && rviz_control_publisher_)
    {
        if (state == Qt::Checked)
            rviz_control_msg_.Loc_match_switch = 1;
        if (state == Qt::Unchecked) // "选中"
            rviz_control_msg_.Loc_match_switch = 2;

        rviz_control_msg_.Upd_match_switch = 0;
        rviz_control_msg_.Links_switch = 0;
        rviz_control_msg_.Iamge_id_switch = 0;
        rviz_control_publisher_.publish(rviz_control_msg_);
        std::cout << "CheckBox_Loc_onStateChanged" << std::endl;
    }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    //   config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    QString topic;
    if (config.mapGetString("Topic", &topic))
    {
        //output_topic_editor_->setText( topic );
        //     updateTopic();
    }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TeleopPanel, rviz::Panel)
// END_TUTORIAL
