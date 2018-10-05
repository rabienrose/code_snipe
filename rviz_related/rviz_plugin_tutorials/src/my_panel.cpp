#include "my_panel.h"

#include "ui_my_panel.h"

#include <QMessageBox>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include "remoteConHelper.h"
#include <iostream>

namespace rviz_plugin_tutorials
{

    MyPanel::MyPanel( QWidget* parent ):rviz::Panel( parent ),ui_(new Ui::Form())
    {
      // set up the GUI
      ui_->setupUi(this);
      labelLocImage = ui_->label;

      sub_img = nh_.subscribe<sensor_msgs::Image>("cur/image", 100, &MyPanel::on_img, this);
      sub_database = nh_.subscribe<sensor_msgs::PointCloud>("database_server", 100, &MyPanel::on_database, this);
      db_ready_notice = nh_.subscribe<std_msgs::String>("db_ready_notice", 100, &MyPanel::on_db_ready_notice, this);
    }

    void MyPanel::on_img(const sensor_msgs::Image::ConstPtr &msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg);
        QImage img = remoteConHelper::getInst()->cvMat2QImage(cv_ptr->image);

        labelLocImage->setPixmap(QPixmap::fromImage(img));
        labelLocImage->setScaledContents(true);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }

    void MyPanel::on_db_ready_notice(const std_msgs::String::ConstPtr &msg)
    {
      static bool isFirstTime = true;
      if(isFirstTime) {
        std::cout << "db ready" << std::endl;
        labelLocImage->setText("DB Ready");
        isFirstTime = false;
      }
    }
    void MyPanel::on_database(const sensor_msgs::PointCloud::ConstPtr &msg)
    {
      static bool isFirstTime = true;
      if(isFirstTime) {
        std::cout << "start loading DB" << std::endl;
        labelLocImage->setText("Loading DB");
        isFirstTime = false;
      }
    }

    // Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
    void MyPanel::save(rviz::Config config) const
    {
      rviz::Panel::save(config);
    }

// Load all configuration data for this panel from the given Config object.
    void MyPanel::load(const rviz::Config &config)
    {
      rviz::Panel::load(config);
    }
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::MyPanel, rviz::Panel)
