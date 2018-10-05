#ifndef REMOTE_CON_PANEL_H
#define REMOTE_CON_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#include <loc_server/rviz_control_msg.h>
#endif

#include <std_msgs/String.h>

class QTextEdit;
class QLabel;
class QComboBox;

namespace rviz_plugin_tutorials
{

class RemoteControl : public rviz::Panel
{
  Q_OBJECT
public:
  QTextEdit *Debug_result;
  QLabel *status_result;
  QLabel *delay_result;
  QLabel *gps_result;
  QLabel *imu_result;
  QComboBox *database_list_cbo;

  RemoteControl(QWidget *parent = 0);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void on_add_database(const std_msgs::String::ConstPtr &msg);
  void on_add_error(const std_msgs::String::ConstPtr &msg);
  void on_add_delay(const std_msgs::String::ConstPtr &msg);
  void on_add_status(const std_msgs::String::ConstPtr &msg);
  void on_imu_status(const std_msgs::String::ConstPtr &msg);
  void on_gps_status(const std_msgs::String::ConstPtr &msg);
  void resetStatus();
public Q_SLOTS:
  void Button_ros_data_sender_event();
  void Button_rosbag_event();
  void Button_ros_loc_algo_event();
  void Button_ros_real_loc_event();
  void database_list_cbo_valueChanged();
  void Button_reboot_event();
  void Button_temp_event();

protected:
  void rebootMac();
  void showMessage(std::string text);
  std::string cur_database;

  ros::Subscriber error_msg_sub;
  ros::Subscriber log_status_sub;
  ros::Subscriber dalay_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber gps_sub;

  ros::NodeHandle nh_;
};

} // end namespace rviz_plugin_tutorials

#endif // REMOTE_CON_PANEL_H
