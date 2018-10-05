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
#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <opencv2/opencv.hpp> // using some utilities of opencv
#include <Eigen/Eigen>

#include <rviz/panel.h>
#endif

#include <QMainWindow>
#include <QDebug>
#include <Qt/qmath.h>
#include <QVector>

#include <std_msgs/String.h>
#include <loc_server/server_img_msg.h>
#include <record_msg/img_record.h>
#include <record_msg/gps_record.h>
#include <record_msg/imu_record.h>
#include <record_msg/imu_pack.h>

class QLineEdit;
class QLabel;
class QComboBox;
class QPushButton;
class QCheckBox;
class ImageWidget;
class MainWindow;
class QGridLayout;
class QPixmap;
class QTextEdit;

namespace rviz_plugin_tutorials
{
// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// record_play will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// ImageWidget class, and is described there.
class record_play : public rviz::Panel
{

  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  QGridLayout *gridLayout;

  QLabel *GPS_Value_Lat;
  QLabel *GPS_Value_Lon;
  QLabel *GPS_Value_Alt;

  QLabel *GPS_ImageLabel;
  QLabel *IMU_ImageLabel;
  QLabel *Camera_ImageLabel;

  QLabel *labelimg;

  QPixmap red_pixmap;
  QPixmap green_pixmap;
  QPixmap yellow_pixmap;

  QPushButton *Button_Record_Start;
  QPushButton *Button_Record_Stop;

  QPushButton *Button_Play_Start;
  QPushButton *Button_Play_Stop;

  QPushButton *Button_Folder;
  QComboBox *ComboBox_Bag;
  QComboBox *ComboBox_Speed;

  QPushButton *Button_Run_sensor_manage;
  QPushButton *Button_Stop_sensor_manage;
  int img_w = 640;
  int img_h = 400;
  int pixmap_width = 20;
  int pixmap_hight = 20;
  int img_miss_num = 0, img_false_num = 0;
  int gps_miss_num = 0, gps_false_num = 0;
  int imu_miss_num = 0, imu_false_num = 0;
  float gps_Lat = 0, gps_Lon = 0, gps_Alt = 0;

  int switch_show_img = 0;
  int img_id = 0;

  bool flag_bag_play = false, flag_record = false, flag_show_img = true;

  ros::Time cur_time, img_time, imu_time, gps_time;

  std::string ComboBox_Bag_currentBag;
  int ComboxBox_Speed = 1;
  std::string ubuntu_rosbag_folder;
  QString Bag_dir;
  std::vector<std::string> bag_list;

  record_play(QWidget *parent = 0);

  void on_msg_image(const record_msg::img_record::ConstPtr &msg);
  void on_msg_imu(const record_msg::imu_pack::ConstPtr &msg);
  void on_msg_gps(const record_msg::gps_record::ConstPtr &msg);

  void show_img(int cur_id, int sum_id, cv::Mat cur_img);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  void Button_Record_Start_event();
  void Button_Record_Stop_event();
  void Button_Play_Start_event();
  void Button_Play_Stop_event();
  void Button_Run_sensor_manage_event();
  void Button_Stop_sensor_manage_event();

  void Button_Record_Loc_info_Start_event();
  void Button_Record_Loc_info_Stop_event();

  void sensor_check();

  void Button_Folder_event();
  void ComboBox_Bag_valueChanged();
  void ComboBox_Speed_valueChanged();

  // Here we declare some internal slots.
protected Q_SLOTS:

  // Then we finish up with protected member variables.
protected:
  ros::Subscriber sub_gps;
  ros::Subscriber sub_img;
  ros::Subscriber sub_imu;

  //     ros::Publisher img_pub;

  ros::NodeHandle nr_;
};

} // end namespace rviz_plugin_tutorials
