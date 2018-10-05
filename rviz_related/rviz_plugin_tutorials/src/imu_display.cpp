#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreVector3.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include "imu_display.h"
#include "rviz/mesh_loader.h"
#include "rviz/display_group.h"
#include "rviz/view_manager.h"
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include "../../rviz_animated_view_controller/include/rviz_animated_view_controller/rviz_animated_view_controller.h"

#include "math.h"

#include <Eigen/Eigen>

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ImuDisplay::ImuDisplay()
{
    color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
                                              "Color to draw the acceleration arrows.",
                                              this, SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
                                              "0 is fully transparent, 1.0 is fully opaque.",
                                              this, SLOT(updateColorAndAlpha()));

    history_length_property_ = new rviz::IntProperty("F", 1100 / 1.4,
                                                     "F of camera.",
                                                     this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(100000);
    cov_on = false;
    cam_pose_on = false;
    backgound_on = true;

    max_local_mp_count = 0;
    show_mp_links = 0;

    show_front_screen = 0;
}

void ImuDisplay::on_rviz_control(const loc_server::rviz_control_msg::ConstPtr &msg)
{
    if (msg->Links_switch == 1)
        show_mp_links = 1;
    if (msg->Links_switch == 2)
        show_mp_links = 0;
    if (msg->His_uncety_switch == 1)
    {
        Displays_predict_uncertainty->setEnabled(true);
        Displays_current_uncertainty->setEnabled(false);
    }
    if (msg->His_uncety_switch == 2)
    {
        Displays_predict_uncertainty->setEnabled(false);
        Displays_current_uncertainty->setEnabled(true);
    }
}

void ImuDisplay::updateViewMode(const loc_server::loc_predict_msg::ConstPtr &msg)
{
    //         predict_bumber++;
    float View_yaw_;
    float View_pitch_;
    float View_distance_;
    Eigen::Quaterniond q;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    //         std::cout<<"euler_result="<<euler.transpose()<<std::endl;
    //         if(3 == view_type_  && predict_bumber > 20)
    //         {
    //             predict_bumber = 0;
    //             rviz::ViewManager* view_mgr=  context_->getViewManager();
    //             rviz::OrbitViewController* ViewController;
    //
    //             view_mgr->setCurrentViewControllerType("rviz/Orbit");
    //             ViewController = (rviz::OrbitViewController*) view_mgr->getCurrent();
    //             ViewController->reset();
    predict_view_yaw_ = euler[0] + 1.6;
    //             View_yaw_ = euler[0] + 1.6;// + 3;
    //             View_pitch_ = 0.3;
    //             View_distance_ = 60;
    //             ViewController->yaw( -View_yaw_  +0.785398);
    //             ViewController->pitch( -View_pitch_  +0.785398);
    //             ViewController->zoom( -View_distance_ +10);
    //         }
}

void ImuDisplay::on_view_control(const loc_server::view_control_msg::ConstPtr &msg)
{
    std::cout<<"on_view_control: "<<std::endl;
    view_type_ = msg->Views_type_choose;
    std::cout << "Views_type_choose = " << view_type_ << std::endl;

    rviz::ViewManager *view_mgr = context_->getViewManager();
    rviz::OrbitViewController *ViewController;
    rviz_animated_view_controller::AnimatedViewController *aViewController;

    float View_distance_;
    float View_yaw_;
    float View_pitch_;
    float View_scale;
    float View_angle;
#ifdef DEV_BUILD
    //Displays_Image->setEnabled(true);
#endif
    switch (msg->Views_type_choose)
    {
        case 0:
            break;
        case 1: //Vehicle_view_FPS:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(true);
            Displays_pointcloud_dense->setEnabled(true);
            Displays_pointcloud_sparse->setEnabled(true);
            Displays_predict_trajectory->setEnabled(true);
            Displays_database_trajectory->setEnabled(false);
            Displays_painting_detected->setEnabled(true);
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(true);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(false);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(true);
            view_mgr->setCurrentViewControllerType("rviz_animated_view_controller/Animated");
            aViewController = (rviz_animated_view_controller::AnimatedViewController *)view_mgr->getCurrent();
            aViewController->setEyePoint(Ogre::Vector3(0.0, 0.0, 0.0));
            aViewController->reset();
            //ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            //ViewController->reset();

            std::cout << "ViewController = Animated FPS" << std::endl;
            break;
        case 2: //Top_own_2Dview:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(false);
            Displays_pointcloud_dense->setEnabled(true);
            Displays_pointcloud_sparse->setEnabled(true);
            Displays_predict_trajectory->setEnabled(true);
            Displays_database_trajectory->setEnabled(false);
            Displays_painting_detected->setEnabled(true);
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(false);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(true);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(true);
            view_mgr->setCurrentViewControllerType("rviz/TopDownOrtho");
            ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            ViewController->reset();
            ViewController->yaw(-8.5);
            ViewController->pitch(-2);
            //ViewController->zoom(1);
            std::cout << "ViewController = TopDownOrtho" << std::endl;
            break;
        case 3: //Free_3Dview:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(true);
            Displays_pointcloud_dense->setEnabled(true);
            Displays_pointcloud_sparse->setEnabled(true);
            Displays_predict_trajectory->setEnabled(true);
            Displays_database_trajectory->setEnabled(true); //----------------------------------
            Displays_painting_detected->setEnabled(true);   //--------------------------------
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(true);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(true);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(true);
            view_mgr->setCurrentViewControllerType("rviz/Orbit");
            ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            ViewController->reset();
            //             View_yaw_ = 0.3;
            View_pitch_ = 0.3;
            View_distance_ = 60;
            ViewController->yaw(-predict_view_yaw_ + 0.785398);
            ViewController->pitch(-View_pitch_ + 0.785398);
            ViewController->zoom(-View_distance_ + 10);
            std::cout << "ViewController = Orbit" << std::endl;
            break;
        case 4: //Debug_mode:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(true);
            Displays_pointcloud_dense->setEnabled(false);
            Displays_pointcloud_sparse->setEnabled(true);
            Displays_predict_trajectory->setEnabled(true);
            Displays_database_trajectory->setEnabled(true);
            Displays_painting_detected->setEnabled(false);
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(true);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(true);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(false);
            view_mgr->setCurrentViewControllerType("rviz/Orbit");
            ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            ViewController->reset();
            View_yaw_ = 0.3;
            View_pitch_ = 0.3;
            View_distance_ = 60;
            ViewController->yaw(-View_yaw_ + 0.785398);
            ViewController->pitch(-View_pitch_ + 0.785398);
            ViewController->zoom(-View_distance_ + 10);
            std::cout << "ViewController = Orbit" << std::endl;
            break;
        case 5: //Simplest_mode:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(false);
            Displays_pointcloud_dense->setEnabled(false);
            Displays_pointcloud_sparse->setEnabled(false);
            Displays_predict_trajectory->setEnabled(false);
            Displays_database_trajectory->setEnabled(false);
            Displays_painting_detected->setEnabled(true);
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(false);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(true);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(true);
            view_mgr->setCurrentViewControllerType("rviz/Orbit");
            ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            ViewController->reset();
            View_yaw_ = 0.3;
            View_pitch_ = 0.3;
            View_distance_ = 60;
            ViewController->yaw(-View_yaw_ + 0.785398);
            ViewController->pitch(-View_pitch_ + 0.785398);
            ViewController->zoom(-View_distance_ + 10);
            std::cout << "ViewController = Orbit" << std::endl;
            break;
        case 6: //Vehicle_view_3rdFollow:
            Displays_Grid->setEnabled(true);
            Displays_TF->setEnabled(false);
            Displays_gps->setEnabled(true);
            Displays_pointcloud_dense->setEnabled(true);
            Displays_pointcloud_sparse->setEnabled(true);
            Displays_predict_trajectory->setEnabled(true);
            Displays_database_trajectory->setEnabled(false);
            Displays_painting_detected->setEnabled(true);
            Displays_painting_manual->setEnabled(false);
            Displays_update_trajectory->setEnabled(true);
            Displays_visual_upd_dir->setEnabled(false);
            Displays_predict_uncertainty->setEnabled(false);
            Displays_current_uncertainty->setEnabled(false);
            Displays_visual_uncertainty->setEnabled(false);
            //Displays_Image->setEnabled(true);
            Displays_car->setEnabled(true);
            view_mgr->setCurrentViewControllerType("rviz_animated_view_controller/Animated");
            aViewController = (rviz_animated_view_controller::AnimatedViewController *)view_mgr->getCurrent();
            aViewController->setEyePoint(Ogre::Vector3(0.0, -10.0, -40.0));
            aViewController->reset();
            // ViewController = (rviz::OrbitViewController *)view_mgr->getCurrent();
            // ViewController->reset();

            std::cout << "ViewController = Animated 3rd Follow" << std::endl;
            break;
        default:
            break;
    }
}

void ImuDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateHistoryLength();
    ros::NodeHandle n;

    show_mp_links = 0;
    view_type_ = 0;
    //         predict_bumber = 0;

    sub_rviz_control = n.subscribe<loc_server::rviz_control_msg>("rviz_control", 100, &ImuDisplay::on_rviz_control, this);
    sub_updateView = n.subscribe<loc_server::loc_predict_msg>("predict_server", 100, &ImuDisplay::updateViewMode, this);
    sub_view_control = n.subscribe<loc_server::view_control_msg>("view_control", 100, &ImuDisplay::on_view_control, this);

    rviz::DisplayGroup *root_display = context_->getRootDisplayGroup();
    int display_count = root_display->numDisplays();
    for (int i = 0; i < display_count; i++)
    {
        Display *display_t = root_display->getDisplayAt(i);
        if (display_t->getNameStd() == "Grid")
            Displays_Grid = display_t;
        if (display_t->getNameStd() == "TF")
            Displays_TF = display_t;
        if (display_t->getNameStd() == "gps")
            Displays_gps = display_t;

        if (display_t->getNameStd() == "pointcloud_dense")
            Displays_pointcloud_dense = display_t;
        if (display_t->getNameStd() == "pointcloud_sparse")
            Displays_pointcloud_sparse = display_t;
        if (display_t->getNameStd() == "predict_trajectory")
            Displays_predict_trajectory = display_t;

        if (display_t->getNameStd() == "database_trajectory")
            Displays_database_trajectory = display_t;
        if (display_t->getNameStd() == "painting_detected")
            Displays_painting_detected = display_t;
        if (display_t->getNameStd() == "painting_manual")
            Displays_painting_manual = display_t;

        if (display_t->getNameStd() == "update_trajectory")
            Displays_update_trajectory = display_t;
        if (display_t->getNameStd() == "visual_upd_dir")
            Displays_visual_upd_dir = display_t;
        if (display_t->getNameStd() == "visual_uncertainty")
            Displays_visual_uncertainty = display_t;

        if (display_t->getNameStd() == "predict_uncertainty")
            Displays_predict_uncertainty = display_t;
        if (display_t->getNameStd() == "current_uncertainty")
            Displays_current_uncertainty = display_t;

        if (display_t->getNameStd() == "car")
            Displays_car = display_t;
        if (display_t->getNameStd() == "Image")
            Displays_Image = display_t;
    }

}

ImuDisplay::~ImuDisplay()
{
}

// Clear the visuals by deleting their objects.
void ImuDisplay::reset()
{
    MFDClass::reset();
}

// Set the current color and alpha values for each visual.
void ImuDisplay::updateColorAndAlpha()
{
}

// Set the number of past visuals to show.
void ImuDisplay::updateHistoryLength()
{
}

void ImuDisplay::update(float wall_dt, float ros_dt)
{
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform("curFrame", ros::Time(), position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", "curFrame", qPrintable(fixed_frame_));
        return;
    }
    if (last_loc == position)
    {
        return;
    }
    else
    {
        last_loc = position;
    }

    if (frame_node_ != NULL)
    {
        if (show_front_screen == 1)
        {
            frame_node_->setVisible(true);
        }
        else
        {
            frame_node_->setVisible(false);
        }
    }
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    int local_mp_count = points.size();
    if (max_local_mp_count < local_mp_count)
    {
        max_local_mp_count = local_mp_count;
    }
    if (local_mp_count > 0)
    {
        if (manual_connection != NULL)
        {
            if (show_mp_links == 0)
            {
                connection_node_->setVisible(false);
                return;
            }
            else
            {
                connection_node_->setVisible(true);
            }
            manual_connection->beginUpdate(0);
            for (int i = 0; i < max_local_mp_count; i++)
            {
                if (i < local_mp_count)
                {
                    manual_connection->position(position);
                    manual_connection->position(points[i]);
                }
                else
                {
                    manual_connection->position(position);
                    manual_connection->position(position);
                }
            }
            manual_connection->end();
        }
        else
        {
            static int count_con = 0;
            manual_connection = context_->getSceneManager()->createManualObject("connection" + boost::lexical_cast<std::string>(count_con));
            manual_connection->estimateVertexCount(10000);
            manual_connection->setDynamic(true);
            manual_connection->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

            for (int i = 0; i < local_mp_count; i++)
            {
                manual_connection->position(position);
                manual_connection->position(points[i]);
            }
            manual_connection->end();
            connection_node_ = scene_node_->getParentSceneNode()->createChildSceneNode();
            connection_node_->attachObject(manual_connection);
        }
    }
}

// This is our callback to handle an incoming message.
void ImuDisplay::processMessage(const veh_msg::veh_msg::ConstPtr &msg)
{

    if (msg->image.height != -1)
    {
        if (manual == NULL)
        {
            static uint32_t count = 0;
            int w = msg->image.width;
            int h = msg->image.height;
            int f = msg->num;
            std::cout << f << std::endl;
            float asp_f = (float)w / 2.0 / f;
            float asp = (float)h / w * asp_f;

            texture = Ogre::TextureManager::getSingleton().createManual(
                "DynamicTexture",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D,
                w, h,
                0,
                Ogre::PF_BYTE_BGRA,
                Ogre::TU_DEFAULT);
            material = Ogre::MaterialManager::getSingleton().create(
                "DynamicTextureMaterial",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

            material->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicTexture");
            material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_COLOUR);

            manual = context_->getSceneManager()->createManualObject("manual" + boost::lexical_cast<std::string>(count++));
            manual->begin("DynamicTextureMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            manual->position(10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(1.0, 0.0);
            manual->position(10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(1.0, 1.0);
            manual->position(-10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(0.0, 1.0);
            manual->position(10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(1.0, 0.0);
            manual->position(-10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(0.0, 1.0);
            manual->position(-10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(0.0, 0.0);

            manual->position(10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(1.0, 0.0);
            manual->position(-10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(0.0, 1.0);
            manual->position(10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(1.0, 1.0);
            manual->position(10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(1.0, 0.0);
            manual->position(-10.0 * asp_f, -10.0 * asp, 10.0);
            manual->textureCoord(0.0, 0.0);
            manual->position(-10.0 * asp_f, 10.0 * asp, 10.0);
            manual->textureCoord(0.0, 1.0);
            manual->end();
            manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
            manual->position(0, 0, 0);
            manual->position(10 * asp_f, -10.0 * asp, 10.0);
            manual->position(0, 0, 0);
            manual->position(10 * asp_f, 10.0 * asp, 10.0);
            manual->position(0, 0, 0);
            manual->position(-10 * asp_f, -10.0 * asp, 10.0);
            manual->position(0, 0, 0);
            manual->position(-10 * asp_f, 10.0 * asp, 10.0);
            manual->end();
            frame_node_ = scene_node_->createChildSceneNode();
            frame_node_->attachObject(manual);

            //std::string mesh_name = "package://visualization_msgs/volks.dae";
            //std::string mesh_name = "package://rviz_plugin_tutorials/media/volks.dae";
            std::string mesh_name = "package://rviz_plugin_tutorials/media/car.dae";
            if (rviz::loadMeshFromResource(mesh_name).isNull())
            {
                std::cout << "could not load [" << mesh_name << "]" << std::endl;
                return;
            }

            std::stringstream ss;
            ss << "mesh_resource_marker_" << count++;
            std::string id = ss.str();
            Ogre::Entity *entity_;
            entity_ = context_->getSceneManager()->createEntity(id, mesh_name);
            //Ogre::SceneNode *car_node_ = scene_node_->createChildSceneNode();
            car_node_ = scene_node_->createChildSceneNode();
            Ogre::Quaternion orientation(0.7071, 0.7071, 0, 0);
            Ogre::Quaternion orientation_z_180(0, 0, 0, 1);
            // float angle = 5;
            // Ogre::Quaternion orientation_row(cos(angle / 2 / 360 * 3.1416), 0, 0, sin(angle / 2 / 360 * 3.1416));
            // orientation = orientation * orientation_z_180 * orientation_row;
            orientation = orientation * orientation_z_180;
            Ogre::Vector3 position(0, 1, 0);
            car_node_->setPosition(position);
            car_node_->setOrientation(orientation);
            car_node_->setScale(Ogre::Vector3(0.85, 0.85, 0.85));
            car_node_->attachObject(entity_);

            // create a default material for any sub-entities which don't have their own.
            // ss << "Material";
            // Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
            // default_material->setReceiveShadows(false);
            // default_material->getTechnique(0)->setLightingEnabled(true);
            // default_material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
            // default_material->getTechnique(0)->setDiffuse(0.3, 0.3, 0.3, 1);
            // entity_->setMaterial(default_material);
        }
        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
        const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

        unsigned char *pDest = static_cast<unsigned char *>(pixelBox.data);
        std::memcpy(pDest, msg->image.data.data(), msg->image.height * msg->image.width * 4);
        // Unlock the pixel buffer
        pixelBuffer->unlock();

        points.clear();
        points.reserve(msg->points.size());
        std::vector<geometry_msgs::Point>::const_iterator it = msg->points.begin();
        std::vector<geometry_msgs::Point>::const_iterator end = msg->points.end();
        for (int i = 0; it != end; ++it, ++i)
        {
            const geometry_msgs::Point &p = *it;
            Ogre::Vector3 v(p.x, p.y, p.z);
            points.push_back(v);
        }
    }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ImuDisplay, rviz::Display)
// END_TUTORIAL
