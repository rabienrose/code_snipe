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
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp> // using some utilities of opencv
#include <Eigen/Eigen>

#include <rviz/panel.h>
#include <loc_server/rviz_control_msg.h>
#include <loc_server/loc_predict_msg.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <record_msg/gps_record.h>
#include <loc_server/loc_update_msg.h>
#include <loc_server/reproj_err_msg.h>
#include <loc_server/view_control_msg.h>
#endif

#include <QMainWindow>
#include <QDebug>
#include <Qt/qmath.h>
#include <QVector>

#include <std_msgs/String.h>

#include <boost/thread/condition.hpp>
#include "remoteConHelper.h"
#include "image_widget.h"

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
class QStackedWidget;

namespace rviz_plugin_tutorials
{
// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// ImageWidget class, and is described there.
class TeleopPanel : public rviz::Panel
{
public:
  static TeleopPanel *getInst()
  {
    static TeleopPanel *me = NULL;
    if (me == NULL)
    {
      me = new TeleopPanel();
    }
    return me;
  }
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  int cv_image_width;
  int cv_image_hight;
  int pixmap_width;
  int pixmap_hight;
  int ComboBox_Database_currentIndex;
  bool show_dgps_info;
  int show_debug_info;
  int start_frameId_flag, start_frameId;

  int img_miss_num, img_false_num;
  int gps_miss_num, gps_false_num;
  int imu_miss_num, imu_false_num;

  bool get_fist_gps_time_flag, button_start_all_flag;
  bool flag_sent_file_name = false;
  bool flag_update_img = false;
  std::string bag_name;

  std::string cur_database;
  std::string prev_database;
  std::string cur_rtv;
  std::string cur_rosbag;
  std::string Record_dir;

  std::string ssh_ros_env;

  QLabel *IMG_MISS;
  QLabel *GPS_MISS;
  QLabel *IMU_MISS;
  QLabel *IMG_FALSE;
  QLabel *GPS_FALSE;
  QLabel *IMU_FALSE;

#ifndef DEV_BUILD
  QPushButton *Button_RestartSrv;
  QPushButton *Button_StartSrv;
  QPushButton *Button_StopSrv;
  QPushButton *Button_QuerySrv;
  QComboBox *ComboBox_Rosbag;
  QPushButton *Button_Play_Start;
  QPushButton *Button_Play_Stop;
  QLabel *status_result;
  QLabel *delay_result;

  ros::Subscriber error_msg_sub;
  ros::Subscriber log_status_sub;
  ros::Subscriber delay_sub;
  QPushButton *Button_SwitchMode;
  QPushButton *Button_Config;
  QPushButton *Button_testSSH;
#endif
  /*QPushButton *Button_Paint;
  QPushButton *Button_Paint_Clear;
  QComboBox *ComboBox_Paint;*/
  QTextEdit *Debug_result;
  QComboBox *ComboBox_View;
  QComboBox *ComboBox_Database;
  QComboBox *ComboBox_Execute;
  QPushButton *Button_Start;
  QPushButton *Button_Stop;
  QPushButton *Button_run_loc;
  QPushButton *Button_Reset;
  QCheckBox *CheckBox_Links;
  QCheckBox *CheckBox_His_uncety;
  QGridLayout *gridLayout;
  QLabel *labelImage;
  QLabel *labelImage_rprj_err;
  QLabel *GPS_ImageLabel;
  QLabel *IMU_ImageLabel;
  QLabel *DGPS_ImageLabel;
  QLabel *labelLocImage;
  ImageWidget *widgetLocImage;
  QStackedWidget *stackedWidget;

  QLabel *Camera_ImageLabel;
  QPixmap red_pixmap;
  QPixmap green_pixmap;
  cv::Mat cv_image;
  cv::Mat cv_image_rprj_err;
  ros::Time cur_time, img_time, imu_time, gps_time;
  void on_show_current_uncertainty(const loc_server::loc_predict_msg::ConstPtr &msg);
  void on_sensor_status(const std_msgs::String::ConstPtr &msg);
  void on_get_first_gps_time(const record_msg::gps_record::ConstPtr &gps_msg);
  void on_gps(const geometry_msgs::Point32::ConstPtr &msg);
  void on_update(const loc_server::loc_update_msg::ConstPtr &msg);
  void on_database(const sensor_msgs::PointCloud::ConstPtr &msg);
  void on_show_rprj_err(const loc_server::reproj_err_msg::ConstPtr &msg);
  void on_new_run(const std_msgs::String::ConstPtr &msg);
  void on_img(const sensor_msgs::Image::ConstPtr &msg);
  void on_db_ready_notice(const std_msgs::String::ConstPtr &msg);

#ifndef DEV_BUILD
  void on_add_error(const std_msgs::String::ConstPtr &msg);
  void on_add_delay(const std_msgs::String::ConstPtr &msg);
  void on_add_status(const std_msgs::String::ConstPtr &msg);
#endif

  void Satrt_recording();
  void Stop_recording();

  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  TeleopPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, ImageWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void setVel(float linear_velocity_, float angular_velocity_);

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic(const QString &topic);

  void sensor_check();
  void Satrt_and_record();
  void CheckBox_FrameId_onStateChanged(int state);
  void CheckBox_Upd_onStateChanged(int state);
  void CheckBox_Loc_onStateChanged(int state);
  void CheckBox_Links_onStateChanged(int state);
  void CheckBox_His_uncety_onStateChanged(int state);
  void CheckBox_rprj_err_onStateChanged(int state);

  void ComboBox_View_valueChanged();
  void ComboBox_Execute_valueChanged();
  void ComboBox_Database_valueChanged();

  void Button_run_loc_event();
  void Button_ReInit_event();
  void Button_QuickInit_event();
  void Button_Start_event();
  void Button_Stop_event();
  void Button_Reset_event();
  void Button_Start_all_event();
  void Button_Stop_all_event();

#ifndef DEV_BUILD
  void Button_StartSrv_event();
  void Button_StopSrv_event();
  void Button_RestartSrv_event();
  void Button_QuerySrv_event();
  void Button_Play_Start_event();
  void Button_Play_Stop_event();
  void ComboBox_Rosbag_valueChanged();
  void Button_SwitchMode_event();
  void Button_Open_config();
  void Button_Test_ssh();
#endif

  /*void Button_Paint_event();
  void Button_Paint_Clear_event();
  void ComboBox_Paint_valueChanged();*/

  // Here we declare some internal slots.
protected Q_SLOTS:

  // Then we finish up with protected member variables.
protected:
  ros::Time img_last_time, img_curt_time;
  ros::Time imu_last_time, imu_curt_time;
  ros::Time gps_last_time, gps_curt_time;
  ros::Time dgps_last_time, dgps_curt_time;

  std::string cur_execute;
  std::string cur_execute_to_kill;
  std::string cur_code;

  std::string ROS_IP;
  std::string algorithm_IP;

  int rprjErrMax;
  int rprjErrMin;
  std::vector<float> current_rprjErr_mean;
  std::vector<float> current_rprjErr_stdd;

  std::vector<float> current_uncertainty;
  std::vector<float> current_error;
  std::map<int, Eigen::Vector3f> current_position;
  std::map<int, Eigen::Vector3f> current_dgps;

  loc_server::rviz_control_msg rviz_control_msg_;
  loc_server::view_control_msg view_control_msg_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_current_uncertainty;
  ros::Subscriber sub_sensor_state;
  ros::Subscriber sub_gps_record_time;
  ros::Subscriber sub_database;
  ros::Subscriber sub_update;
  ros::Subscriber sub_rprj_err;
  ros::Subscriber sub_new_run;
  ros::Subscriber sub_gps;
  ros::Subscriber sub_img;
  ros::Subscriber db_ready_notice;

  ros::Publisher paint_publisher_;
  ros::Publisher rviz_control_publisher_;
  ros::Publisher publisher_stop_algo;
  ros::Publisher publisher_file_name;
  ros::Publisher view_control_publisher_;
  ros::Publisher publisher_run_loc;
  ros::Publisher publisher_Reinit_loc;
  ros::Publisher publisher_Quickinit_loc;

  // The latest velocity values from the drive widget.
  float linear_velocity_;
  float angular_velocity_;
  // END_TUTORIAL

private:
    void InitView();
    remoteConHelper *con;
};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
