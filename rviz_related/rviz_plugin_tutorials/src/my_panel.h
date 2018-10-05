#ifndef MY_PANEL_H
#define MY_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>

#endif

#include <QLabel>

namespace Ui
{
class Form;
}

namespace rviz_plugin_tutorials
{

class MyPanel: public rviz::Panel
{
Q_OBJECT
public:
  MyPanel( QWidget* parent = 0 );

  ~MyPanel() {}

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

  void on_database(const sensor_msgs::PointCloud::ConstPtr &msg);
  void on_db_ready_notice(const std_msgs::String::ConstPtr &msg);
  void on_img(const sensor_msgs::Image::ConstPtr &msg);
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_img;
  ros::Subscriber sub_database;
  ros::Subscriber db_ready_notice;

  Ui::Form *ui_;
  QLabel *labelLocImage;

private Q_SLOTS:
}; // MyPanel

}  // my_panel
#endif // MY_PANEL_H
