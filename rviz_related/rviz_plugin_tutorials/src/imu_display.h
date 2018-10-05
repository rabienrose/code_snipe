#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <veh_msg/veh_msg.h>
#include <OGRE/OgreMaterialManager.h>
#include <loc_server/rviz_control_msg.h>
#include <loc_server/loc_predict_msg.h>
#include <loc_server/view_control_msg.h>
#endif

namespace Ogre
{
class SceneNode;
class ManualObject;
class TexturePtr;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_tutorials
{
class ImuDisplay : public rviz::MessageFilterDisplay<veh_msg::veh_msg>
{
  Q_OBJECT
public:
  int show_mp_links;
  int show_front_screen;
  int view_type_;
  //     int predict_bumber;
  float predict_view_yaw_ = 0;

  ros::Subscriber sub_view_control;
  ros::Subscriber sub_rviz_control;
  ros::Subscriber sub_updateView;

  void on_view_control(const loc_server::view_control_msg::ConstPtr &msg);
  void on_rviz_control(const loc_server::rviz_control_msg::ConstPtr &msg);
  void updateViewMode(const loc_server::loc_predict_msg::ConstPtr &msg);
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ImuDisplay();
  virtual ~ImuDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
  virtual void update(float wall_dt, float ros_dt);

protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const veh_msg::veh_msg::ConstPtr &msg);

  // User-editable property variables.
  rviz::ColorProperty *color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::IntProperty *history_length_property_;
  Ogre::ManualObject *manual = NULL;
  Ogre::ManualObject *manual_connection = NULL;
  Ogre::SceneNode *frame_node_ = NULL;
  Ogre::SceneNode *connection_node_ = NULL;
  Ogre::SceneNode *car_node_ = NULL;
  Ogre::MaterialPtr material;
  Ogre::TexturePtr texture;
  std::vector<Ogre::Vector3> points;
  Ogre::Vector3 last_loc;
  Ogre::Vector3 last_local_mp_count;
  bool cov_on;
  bool cam_pose_on;
  bool backgound_on;

  Display *Displays_Grid;
  Display *Displays_TF;
  Display *Displays_gps;
  Display *Displays_pointcloud_dense;
  Display *Displays_pointcloud_sparse;
  Display *Displays_predict_trajectory;
  Display *Displays_database_trajectory;
  Display *Displays_painting_detected;
  Display *Displays_painting_manual;
  Display *Displays_update_trajectory;
  Display *Displays_visual_upd_dir;
  Display *Displays_visual_uncertainty;
  Display *Displays_predict_uncertainty;
  Display *Displays_current_uncertainty;
  Display *Displays_car;
  Display *Displays_Image;

  int max_local_mp_count;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // IMU_DISPLAY_H
// %EndTag(FULL_SOURCE)%
