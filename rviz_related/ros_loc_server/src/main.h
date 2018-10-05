#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <loc_server/loc_predict_msg.h>
#include <loc_server/loc_update_msg.h>
#include <loc_server/match_info_msg.h>
#include <loc_server/server_img_msg.h>
#include <loc_server/rviz_control_msg.h>
#include <cv_bridge/cv_bridge.h>
#include <veh_msg/veh_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <record_msg/data_record.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/String.h>

class localization_server
{
private:
    void clearBuffer(const int current_frameId);
  public:
    localization_server();
    void on_packer(const record_msg::data_record::ConstPtr &msg);
    void on_predict(const loc_server::loc_predict_msg::ConstPtr &msg);
    void on_update(const loc_server::loc_update_msg::ConstPtr &msg);
    void on_gps(const geometry_msgs::Point32::ConstPtr &msg);
    void on_image(const loc_server::server_img_msg::ConstPtr &msg);
    void on_rviz_control(const loc_server::rviz_control_msg::ConstPtr &msg);
    void on_matches(const loc_server::match_info_msg::ConstPtr &msg);
    void on_database(const sensor_msgs::PointCloud::ConstPtr &msg);
    void on_kfposition(const sensor_msgs::PointCloud::ConstPtr &msg);
    void on_display(const int current_frameId_);
    void DrawImage(cv::Mat &curKfImg);
    std::vector<cv::Point2f> calculate_reProjection(std::vector<Eigen::Vector3f> mapPoint, Eigen::Vector3f position, Eigen::Quaternionf direction);

    bool isInROIArea(cv::Point2f& point);
    void showROIArea(cv::Mat &img);
    void checkROI(std::string roi_area);
    bool fullROI = false;
    int roi_x0, roi_y0, roi_x1, roi_y1;
    bool useROI = false;
    bool showROI = false;
    std::string ROI_area1;
    std::string ROI_area2;
    std::string ROI_area3;

    int max_frame_size = 10000;
    int update_cloud_count = 0;
    float scale_down = 1;

    ros::Subscriber sub_predict;
    ros::Subscriber sub_update;
    ros::Subscriber sub_match;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_image;
    ros::Subscriber sub_database;
    ros::Subscriber sub_kfposition;
    ros::Subscriber sub_rviz_control;
    ros::Subscriber sub_packer;

    ros::Publisher imu_traj_pub;
    ros::Publisher update_traj_pub;
    ros::Publisher gps_pub;
    ros::Publisher marker_pub;
    ros::Publisher mps_pub;
    ros::Publisher kfposition_pub;
    ros::Publisher marker_line_pub;
    ros::Publisher marker_imu_pub;
    ros::Publisher marker_vis_pub;
    ros::Publisher posi_show_mp_pub;
    ros::Publisher imu_indicator_pub;
    ros::Publisher gps_indicator_pub;
    ros::Publisher start_newRun_pub;

    ros::Publisher current_uncertainty;

    ros::Publisher img_pub;
    tf2_ros::TransformBroadcaster curPose;

    ros::Time current_time, start_time;
    ros::Time time_receive_gps, time_receive_predict;

    //------global
    bool need_refresh_img;
    int Loc_match_switch = 0, Upd_match_switch = 1, Image_id_switch = 1;

    int current_frameId, last_frameId, current_pakerId, first_img_flag, first_time_flag, start_frameId = 0;
    bool need_clear_map;
    bool need_clear_kf;
    bool read_img_from_ros;
    float resize_scale, cx, cy, fx, fy;
    int img_ori_w, img_ori_h;
    int img_w = 640;
    int img_h = 400;
    int imu_count;
    int gps_count;

    //------img
    std::map<int, cv::Mat> map_img;
    //------loc_match
    std::map<int, std::vector<cv::KeyPoint>> map_loc_match_kp;
    std::map<int, std::vector<Eigen::Vector3f>> map_loc_match_mp;
    //------update
    std::map<int, std::vector<int> > map_inlier_vec;
    std::map<int, std::vector<cv::KeyPoint>> map_upd_match_kp;
    std::map<int, std::vector<Eigen::Vector3f>> map_upd_match_mp;
    std::map<int, Eigen::Vector3f> map_upd_posi;
    std::map<int, Eigen::Vector3f> map_upd_posi_vis;
    std::map<int, Eigen::Quaternionf> map_upd_dir;
    std::map<int, Eigen::Vector3f> map_upd_cov;
    std::map<int, float> upd_cov_vis_map;
    //------predict
    std::map<int, Eigen::Vector3f> map_pred_posi;
    std::map<int, Eigen::Quaternionf> map_pred_dir;
    std::map<int, Eigen::Vector3f> map_pred_cov;
    //------vec_gps_posi
    std::vector<Eigen::Vector3f> vec_gps_posi;

    visualization_msgs::MarkerArray imu_states;
    visualization_msgs::Marker update_traj_msg;
    visualization_msgs::MarkerArray vis_states;

    //     std::vector<Eigen::Vector3f> mp_sparse_list;

    sensor_msgs::PointCloud database_mp;
    sensor_msgs::PointCloud kf_mp;
    ros::Time cur_time;
};
