#include "main.h"
#include "../../rviz_plugin_tutorials/src/context.h"
#include <thread>
#include <mutex>

std::mutex g_i_mutex; // protects g_i

template <typename T>
void map_delet(std::map<int, T> &input, int key)
{
    if (key > 100)
    {
        auto it = input.begin();
        while (it != input.end())
        {
            if (it->first < (key - 100))
            {
                input.erase(it++);
            }
            else
            {
                it++;
            }
        }
    }
}
void localization_server::clearBuffer(const int current_frameId_)
{
    static int last_clear_frame_id = -1;

    if (current_frameId_ == 0)
    {
        std::cout << "clear all buffer data." << std::endl;
        last_clear_frame_id = -1;

        map_img.clear();
        map_inlier_vec.clear();
        map_loc_match_kp.clear();
        map_loc_match_mp.clear();
        map_upd_match_kp.clear();
        map_upd_match_mp.clear();
        map_upd_posi.clear();
        map_upd_posi_vis.clear();
        map_upd_dir.clear();
        map_upd_cov.clear();
        upd_cov_vis_map.clear();
        map_pred_posi.clear();
        map_pred_dir.clear();
        map_pred_cov.clear();
        vec_gps_posi.clear();
    }
    else
    {
        //clear buffer every 60 frames
        if (current_frameId_ - last_clear_frame_id >= 100)
        {
            //std::cout << "clear buffer data less than:" << current_frameId_ << std::endl;
            map_delet(map_img, current_frameId_);
            map_delet(map_loc_match_kp, current_frameId_);
            map_delet(map_loc_match_mp, current_frameId_);
            map_delet(map_upd_match_kp, current_frameId_);
            map_delet(map_inlier_vec, current_frameId_);
            map_delet(map_upd_match_mp, current_frameId_);
            //map_delet(map_upd_posi, current_frameId_);
            map_delet(map_upd_dir, current_frameId_);
            map_delet(map_upd_cov, current_frameId_);
            map_delet(upd_cov_vis_map, current_frameId_);
            //map_delet(map_pred_posi, current_frameId_);
            map_delet(map_pred_dir, current_frameId_);
            map_delet(map_pred_cov, current_frameId_);

            last_clear_frame_id = current_frameId_;
        }
    }
}

std::vector<cv::Point2f> localization_server::calculate_reProjection(std::vector<Eigen::Vector3f> mapPoint,
                                                                     Eigen::Vector3f position,
                                                                     Eigen::Quaternionf direction)
{

    std::vector<cv::Point2f> reProjection_point;
    Eigen::Matrix<float, 3, 4> parameter_K;
    parameter_K << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0;
    Eigen::Matrix<float, 4, 4> TMatrix;
    TMatrix << direction.toRotationMatrix(), position, 0, 0, 0, 1;

    for (int i = 0; i < mapPoint.size(); i++)
    {
        Eigen::Vector3f rePojrctPosi3f;
        Eigen::Vector4f mapPoint_temp;
        mapPoint_temp << mapPoint[i], 1;
        Eigen::Vector4f cam_mp = (TMatrix.inverse()) * mapPoint_temp;
        rePojrctPosi3f = parameter_K * (TMatrix.inverse()) * mapPoint_temp;
        cv::Point2f rePojrctPosi2f;
        rePojrctPosi2f.x = rePojrctPosi3f[0] / rePojrctPosi3f[2] * img_w / img_ori_w;
        rePojrctPosi2f.y = rePojrctPosi3f[1] / rePojrctPosi3f[2] * img_h / img_ori_h;
        reProjection_point.push_back(rePojrctPosi2f);
    }

    return reProjection_point;
}
void localization_server::DrawImage(cv::Mat &oriKfImg)
{
    cv::Mat chanel4KfImg;
    cv::cvtColor(oriKfImg, chanel4KfImg, CV_BGR2BGRA);
    //img_=curKfImg.rowRange(0, curKfImg.rows);
    cv::resize(chanel4KfImg, chanel4KfImg, cv::Size(chanel4KfImg.cols, chanel4KfImg.rows));
    need_refresh_img = true;
    //        cv::flip(curKfImg, curKfImg,1);
    sensor_msgs::Image rviz_img;
    rviz_img.height = chanel4KfImg.rows;
    rviz_img.width = chanel4KfImg.cols;
    rviz_img.encoding = "8UC4";
    unsigned char *p = chanel4KfImg.ptr<unsigned char>(0);
    std::vector<unsigned char> vec(p, p + chanel4KfImg.cols * 4 * chanel4KfImg.rows);
    rviz_img.data = vec;
    rviz_img.step = chanel4KfImg.cols * 4;
    img_pub.publish(rviz_img);

    return;
}
void DrawCross(cv::Mat &img, cv::Point2d point, cv::Scalar color, int size, int thickness)
{
    cv::line(img, cv::Point2d(point.x - size / 2, point.y), cv::Point2d(point.x + size / 2, point.y), color, thickness);
    cv::line(img, cv::Point2d(point.x, point.y - size / 2), cv::Point2d(point.x, point.y + size / 2), color, thickness);
    return;
}

void DrawTransRec(cv::Mat &img, int x, int y, int width, int height, cv::Scalar color, double alpha)
{
    IplImage img_i(img);
    IplImage *rec = cvCreateImage(cvSize(width, height), img_i.depth, img_i.nChannels);
    cvRectangle(rec, cvPoint(0, 0), cvPoint(width, height), color, -1);
    cvSetImageROI(&img_i, cvRect(x, y, width, height));
    cvAddWeighted(&img_i, alpha, rec, 1 - alpha, 0.0, &img_i);
    cvResetImageROI(&img_i);

    cvReleaseImage(&rec);

    return;
}
void localization_server::on_packer(const record_msg::data_record::ConstPtr &msg)
{
    if (msg->img.size() > 0)
    {
        cv::Mat temp_img;
        temp_img = cv::imdecode(msg->img[0].vecCharImage, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
        cv::resize(temp_img, temp_img, cv::Size(temp_img.cols / 4, temp_img.rows / 4));
        cv::imshow("raw img", temp_img);
        cv::waitKey(1);
    }
    if (msg->gps.size() > 0)
    {
        gps_count = gps_count + msg->gps.size();
        std_msgs::String string_msg;
        std::stringstream ss;
        ss << gps_count;
        string_msg.data = ss.str();
        gps_indicator_pub.publish(string_msg);
    }

    if (msg->imu_gyro.size() > 0)
    {
        imu_count = imu_count + msg->imu_gyro.size();
        std_msgs::String string_msg;
        std::stringstream ss;
        ss << imu_count;
        string_msg.data = ss.str();
        imu_indicator_pub.publish(string_msg);
    }
}

void localization_server::on_predict(const loc_server::loc_predict_msg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    //    need_clear_map = true;
    time_receive_predict = ros::Time::now();
    if (first_img_flag == 0)
    {
        start_frameId = msg->frame_id;
        first_img_flag = 1;
        //std::cout<<"start_frameId(from on_predict)="<<start_frameId<<std::endl;
        if (first_time_flag == 0)
        {
            start_time = ros::Time::now();
            first_time_flag = 1;
            std::cout << "start_time(from on_predict)." << std::endl;
        }
    }
    int frame_id = msg->frame_id;

    geometry_msgs::Point posi = msg->pose.position;
    posi_show_mp_pub.publish(posi);
    map_pred_posi[frame_id] = Eigen::Vector3f(posi.x, posi.y, posi.z);

    geometry_msgs::Quaternion dir = msg->pose.orientation;
    map_pred_dir[frame_id].x() = dir.x;
    map_pred_dir[frame_id].y() = dir.y;
    map_pred_dir[frame_id].z() = dir.z;
    map_pred_dir[frame_id].w() = dir.w;

    map_pred_cov[frame_id] = Eigen::Vector3f(msg->cov.x, msg->cov.y, msg->cov.z);

    visualization_msgs::Marker current_imu_state;
    current_imu_state.header.frame_id = "/world";
    current_imu_state.header.stamp = ros::Time::now();
    current_imu_state.ns = "roadPaint";
    current_imu_state.id = 0;
    current_imu_state.type = visualization_msgs::Marker::SPHERE;
    current_imu_state.action = visualization_msgs::Marker::ADD;
    current_imu_state.pose.position.x = msg->pose.position.x;
    current_imu_state.pose.position.y = msg->pose.position.y;
    current_imu_state.pose.position.z = msg->pose.position.z;
    current_imu_state.pose.orientation.x = 0;
    current_imu_state.pose.orientation.y = 0;
    current_imu_state.pose.orientation.z = 0;
    current_imu_state.pose.orientation.w = 1;
    current_imu_state.scale.x = sqrt(msg->cov.x) * 3 * 10;
    current_imu_state.scale.y = sqrt(msg->cov.y) * 3 * 10;
    current_imu_state.scale.z = sqrt(msg->cov.z) * 3 * 10;
    if (current_imu_state.scale.x > 10)
        current_imu_state.scale.x = 10;
    if (current_imu_state.scale.y > 10)
        current_imu_state.scale.y = 10;
    if (current_imu_state.scale.z > 10)
        current_imu_state.scale.z = 10;
    //std::cout<<current_imu_state.scale<<std::endl;
    current_imu_state.color.r = 0.0f;
    current_imu_state.color.g = 1.0f;
    current_imu_state.color.b = 0.0f;
    current_imu_state.color.a = 0.5;
    current_imu_state.lifetime = ros::Duration();
    current_imu_state.header.stamp = ros::Time::now();
    current_uncertainty.publish(current_imu_state);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "curFrame";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = posi.x;
    transformStamped.transform.translation.y = posi.y;
    transformStamped.transform.translation.z = posi.z;
    transformStamped.transform.rotation.x = dir.x;
    transformStamped.transform.rotation.y = dir.y;
    transformStamped.transform.rotation.z = dir.z;
    transformStamped.transform.rotation.w = dir.w;

    curPose.sendTransform(transformStamped);

    sensor_msgs::PointCloud traj_imu_msg;
    traj_imu_msg.header.frame_id = "/world";
    traj_imu_msg.points.resize(map_pred_posi.size());
    int traj_node_count = 0;
    for (auto posi_it = map_pred_posi.begin(); posi_it != map_pred_posi.end(); posi_it++)
    {
        traj_imu_msg.points[traj_node_count].x = posi_it->second.x();
        traj_imu_msg.points[traj_node_count].y = posi_it->second.y();
        traj_imu_msg.points[traj_node_count].z = posi_it->second.z();
        traj_node_count++;
    }
    imu_traj_pub.publish(traj_imu_msg);

    int cov_count = 0;
    imu_states.markers.clear();
    for (auto cov_it = map_pred_cov.rbegin(); cov_it != map_pred_cov.rend(); cov_it++)
    {
        if (cov_count > 100)
        {
            break;
        }
        float cov = sqrt(cov_it->second.x() * cov_it->second.x() + cov_it->second.y() * cov_it->second.y() + cov_it->second.z() * cov_it->second.z());
        visualization_msgs::Marker imu_state;
        imu_state.header.frame_id = "/world";
        imu_state.header.stamp = ros::Time::now();
        imu_state.ns = "roadPaint";
        imu_state.id = cov_count;
        imu_state.type = visualization_msgs::Marker::SPHERE;
        imu_state.action = visualization_msgs::Marker::ADD;
        auto it_pre_posi = map_pred_posi.find(cov_it->first);
        if (it_pre_posi == map_pred_posi.end())
        {
            continue;
        }
        Eigen::Vector3f his_posi = it_pre_posi->second;
        imu_state.pose.position.x = his_posi.x();
        imu_state.pose.position.y = his_posi.y();
        imu_state.pose.position.z = his_posi.z();
        imu_state.pose.orientation.x = 0;
        imu_state.pose.orientation.y = 0;
        imu_state.pose.orientation.z = 0;
        imu_state.pose.orientation.w = 1;
        float imu_cov_scale = cov * 100;
        if (imu_cov_scale > 10)
        {
            imu_cov_scale = 10;
        }

        imu_state.scale.x = imu_cov_scale;
        imu_state.scale.y = imu_cov_scale;
        imu_state.scale.z = imu_cov_scale;
        imu_state.color.r = 0.0f;
        imu_state.color.g = 1.0f;
        imu_state.color.b = 0.0f;
        imu_state.color.a = 0.5;
        imu_state.lifetime = ros::Duration();
        imu_state.header.stamp = ros::Time::now();
        imu_states.markers.push_back(imu_state);
        cov_count++;
    }
    marker_imu_pub.publish(imu_states);

    veh_msg::veh_msg marker_img;
    marker_img.header.frame_id = "/curFrame";
    marker_img.num = 0;
    marker_img.header.stamp = ros::Time::now();
    sensor_msgs::Image rviz_img;
    rviz_img.height = -1;
    marker_img.image = rviz_img;
    marker_pub.publish(marker_img);
}
static std::vector<std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}
void localization_server::checkROI(std::string roi_area)
{
    useROI = "True" == LocContext::getInstance()->getParam("Control", "switch_use_ROI");
    if(useROI) {
        std::vector <std::string> vals = split(roi_area, ',');
        if (vals.size() == 4) {
            int x0 = std::stoi(vals[0]);
            int y0 = std::stoi(vals[1]);
            int w = std::stoi(vals[2]);
            int h = std::stoi(vals[3]);

            if (x0 < 0)
                x0 = 0;
            if (x0 > img_w)
                x0 = img_w;

            if (y0 < 0)
                y0 = 0;
            if (y0 > img_h)
                y0 = img_h;

            if (w < 0)
                w = 0;
            if (w > img_w)
                w = img_w;

            if (h < 0)
                h = 0;
            if (h > img_h)
                h = img_h;


            if (x0 == 0 && y0 == 0 && w == img_w && h == img_h) {
                fullROI = true;
            }

            roi_x0 = x0;
            roi_y0 = y0;
            roi_x1 = x0 + w;
            roi_y1 = y0 + h;
        }
    }
}
bool localization_server::isInROIArea(cv::Point2f& point)
{
    useROI = "True" == LocContext::getInstance()->getParam("Control", "switch_use_ROI");
    ROI_area1 = LocContext::getInstance()->getParam("Control", "ROI_area1");
    ROI_area2 = LocContext::getInstance()->getParam("Control", "ROI_area2");
    ROI_area3 = LocContext::getInstance()->getParam("Control", "ROI_area3");
    if(useROI) {
        // display outliers if they lie in ROI area
        checkROI(ROI_area1);
        if(fullROI)
            return true;
        if(roi_x0 == roi_x1 && roi_y0 == roi_y1)
            return false;
        bool ret1 = point.x >= roi_x0 && point.x <= roi_x1 && point.y >= roi_y0 && point.y <= roi_y1;
        if(ret1)
            return true;

        checkROI(ROI_area2);
        if(fullROI)
            return true;
        if(roi_x0 == roi_x1 && roi_y0 == roi_y1)
            return false;
        bool ret2 = point.x >= roi_x0 && point.x <= roi_x1 && point.y >= roi_y0 && point.y <= roi_y1;
        if(ret2)
            return true;

        checkROI(ROI_area3);
        if(fullROI)
            return true;
        if(roi_x0 == roi_x1 && roi_y0 == roi_y1)
            return false;
        bool ret3 = point.x >= roi_x0 && point.x <= roi_x1 && point.y >= roi_y0 && point.y <= roi_y1;
        if(ret3)
            return true;

        return false;
    }
    return false;
}
void localization_server::showROIArea(cv::Mat &img)
{
    useROI = "True" == LocContext::getInstance()->getParam("Control", "switch_use_ROI");
    showROI = "True" == LocContext::getInstance()->getParam("Control", "show_ROI");
    ROI_area1 = LocContext::getInstance()->getParam("Control", "ROI_area1");
    ROI_area2 = LocContext::getInstance()->getParam("Control", "ROI_area2");
    ROI_area3 = LocContext::getInstance()->getParam("Control", "ROI_area3");
    if(useROI) {
        const cv::Scalar colorBlack(0, 0, 0);
        checkROI(ROI_area1);
        if(showROI)
            DrawTransRec(img, roi_x0, roi_y0, roi_x1 - roi_x0, roi_y1 - roi_y0, colorBlack, 0.8);

        checkROI(ROI_area2);
        if(showROI)
            DrawTransRec(img, roi_x0, roi_y0, roi_x1 - roi_x0, roi_y1 - roi_y0, colorBlack, 0.8);

        checkROI(ROI_area3);
        if(showROI)
            DrawTransRec(img, roi_x0, roi_y0, roi_x1 - roi_x0, roi_y1 - roi_y0, colorBlack, 0.8);
    }
}
void localization_server::on_display(int current_frameId_)
{
    useROI = "True" == LocContext::getInstance()->getParam("Control", "switch_use_ROI");
    typedef std::map<int, std::vector<cv::KeyPoint>> key_point_map;
    key_point_map::iterator it_loc_match = map_loc_match_kp.find(current_frameId_);
    if (it_loc_match != map_loc_match_kp.end() || read_img_from_ros)
    {
        key_point_map::iterator it_update_match = map_upd_match_kp.find(current_frameId_);
        if (it_update_match != map_upd_match_kp.end())
        {
            typedef std::map<int, cv::Mat> current_map_img;
            if (read_img_from_ros == true)
            {
                cv::Mat img_from_ros;
                std::string img_name = LocContext::getInstance()->getParam("Control", "img_path_when_run_Asample") + std::to_string(current_frameId_) + ".jpg";
                img_from_ros = cv::imread(img_name);
                if (img_from_ros.empty())
                    return;
                cv::resize(img_from_ros, img_from_ros, cv::Size(img_w, img_h));
                map_img[current_frameId_] = img_from_ros;
            }
            current_map_img::iterator map_img_ = map_img.find(current_frameId_);
            if (map_img_ != map_img.end())
            {
                std::stringstream text;
                std::vector<cv::KeyPoint> upd_keypoint, loc_keypoint;
                loc_keypoint = it_loc_match->second;
                upd_keypoint = it_update_match->second;

                std::vector<int> vecInlierIdx;
                std::vector<bool> vecIsInlier = std::vector<bool>();
                vecIsInlier.resize(upd_keypoint.size(), false);
                if(map_inlier_vec.find(current_frameId_) != map_inlier_vec.end()) {
                    vecInlierIdx = map_inlier_vec[current_frameId_];
                    if(vecIsInlier.size() > vecInlierIdx.size()) {
                        for (int i = 0; i < vecInlierIdx.size(); ++i) {
                            if (0 < vecInlierIdx[i] && vecInlierIdx[i] < vecIsInlier.size())
                                vecIsInlier[vecInlierIdx[i]] = true;
                        }
                    }
                }

                const cv::Scalar colorGreen(0, 255, 0);
                const cv::Scalar colorCyan(255, 255, 0);
                const cv::Scalar colorBlue(255, 0, 0);
                const cv::Scalar colorRed(0, 0, 255);
                const cv::Scalar colorYellow(0, 255, 255);
                const cv::Scalar colorGray(200, 200, 200);
                const cv::Scalar colorBlack(0, 0, 0);
                cv::Mat img_ = map_img_->second.clone();

                showROIArea(map_img_->second);
                if (1 == Loc_match_switch && (false == read_img_from_ros))
                {
                    //loc_keypoint = it_loc_match->second;
                    if (Upd_match_switch == 1)
                        DrawTransRec(map_img_->second, 0, 0, 130, 40, colorBlack, 0.8);
                    else
                        DrawTransRec(map_img_->second, 0, 0, 130, 20, colorBlack, 0.8);
                    text << ":loc_match";
                    cv::putText(map_img_->second, text.str(), cv::Point2d(20, 15), cv::FONT_HERSHEY_TRIPLEX, 0.5, colorBlue, 1);
                    cv::circle(map_img_->second, cv::Point2d(10, 10), 7, colorBlue, 1.5);
                    DrawCross(map_img_->second, cv::Point2d(10, 10), colorBlue, 6, 2);
                    text.str("");
                    for (int i = 0; i < loc_keypoint.size(); i++)
                    {
                        cv::circle(map_img_->second, loc_keypoint[i].pt, 7, colorBlue, 1.5);
                    }
                    std::vector<cv::Point2f> loc_reProj_point = localization_server::calculate_reProjection(map_loc_match_mp[current_frameId_],
                                                                                                            map_upd_posi[current_frameId_],
                                                                                                            map_upd_dir[current_frameId_]);
                    for (int i = 0; i < loc_reProj_point.size(); i++)
                    {
                        DrawCross(map_img_->second, loc_reProj_point[i], colorBlue, 6, 2);
                        cv::line(map_img_->second, loc_reProj_point[i], loc_keypoint[i].pt, colorBlue, 1, 8, 0);
                    }
                }
                if (Upd_match_switch == 1)
                {
                    //upd_keypoint = it_update_match->second;
                    text << ":upd_match";
                    if (Loc_match_switch == 1)
                    {
                        cv::circle(map_img_->second, cv::Point2d(10, 30), 7, colorGreen, 1.5);
                        DrawCross(map_img_->second, cv::Point2d(10, 30), colorGreen, 6, 2);
                        cv::putText(map_img_->second, text.str(), cv::Point2d(20, 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, colorGreen, 1);
                    }
                    else
                    {
                        DrawTransRec(map_img_->second, 0, 0, 130, 20, colorBlack, 0.8);
                        cv::circle(map_img_->second, cv::Point2d(10, 10), 7, colorGreen, 1.5);
                        DrawCross(map_img_->second, cv::Point2d(10, 10), colorGreen, 6, 2);
                        cv::putText(map_img_->second, text.str(), cv::Point2d(20, 15), cv::FONT_HERSHEY_TRIPLEX, 0.5, colorGreen, 1);
                    }

                    text.str("");
                    std::vector<cv::Point2f> upd_reProj_point = localization_server::calculate_reProjection(map_upd_match_mp[current_frameId_],
                                                                                                            map_upd_posi[current_frameId_], map_upd_dir[current_frameId_]);
                    for (int i = 0; i < upd_keypoint.size(); i++)
                    {
                        /*if(true == vecIsInlier[i] || (config.switch_use_ROI && isInROIArea(upd_keypoint[i].pt))) {
                            cv::circle(map_img_->second, upd_keypoint[i].pt, 7, colorGreen, 1.5);
                            cv::circle(img_, upd_keypoint[i].pt, 2, colorRed, 4);
                            DrawCross(map_img_->second, upd_reProj_point[i], colorGreen, 6, 2);
                            cv::line(map_img_->second, upd_reProj_point[i], upd_keypoint[i].pt, colorGreen, 1, 8, 0);
                        }*/
                        if(true == vecIsInlier[i]) {
                            cv::circle(map_img_->second, upd_keypoint[i].pt, 7, colorGreen, 1.5);
                            cv::circle(img_, upd_keypoint[i].pt, 2, colorRed, 4);
                            DrawCross(map_img_->second, upd_reProj_point[i], colorGreen, 6, 2);
                            cv::line(map_img_->second, upd_reProj_point[i], upd_keypoint[i].pt, colorGreen, 1, 8, 0);
                        } else if(useROI && isInROIArea(upd_keypoint[i].pt))
                        {
                            showROI = "True" == LocContext::getInstance()->getParam("Control", "show_ROI");

                            cv::circle(map_img_->second, upd_keypoint[i].pt, 7, showROI ? colorRed : colorGreen, 1.5);
                            DrawCross(map_img_->second, upd_keypoint[i].pt, showROI ? colorRed : colorGreen, 6, 2);
                            cv::circle(img_, upd_keypoint[i].pt, 2, showROI ? colorRed : colorGreen, 4);


                            //DrawCross(map_img_->second, upd_reProj_point[i], colorRed, 6, 2);
                            //cv::line(map_img_->second, upd_reProj_point[i], upd_keypoint[i].pt, colorRed, 1, 8, 0);
                        }
                    }
                }
                if (Image_id_switch == 1)
                {
                    current_time = ros::Time::now();
                    current_pakerId = start_frameId + (int)((current_time.toSec() - start_time.toSec()) * 30);
                    DrawTransRec(map_img_->second, 0, 378, img_w, 22, colorBlack, 0.8);
                    text << "frmId/pkrId=" << current_frameId_ << "/" << current_pakerId;
                    text << ",skip=" << current_frameId_ - last_frameId;
                    if (read_img_from_ros == false && Loc_match_switch == 1)
                        text << ",loc_mt=" << loc_keypoint.size();
                    text << ",upd_mt=" << vecInlierIdx.size();
                    cv::putText(map_img_->second, text.str(), cv::Point2d(3, map_img_->second.rows - 5), cv::FONT_HERSHEY_TRIPLEX, 0.6, colorYellow, 1);
                }
                DrawImage(map_img_->second);
                last_frameId = current_frameId_;

                //map_loc_match_kp.erase(map_loc_match_kp.begin(), map_loc_match_kp.lower_bound(current_frameId_));
                clearBuffer(current_frameId_);

                veh_msg::veh_msg marker_img;
                cv::cvtColor(img_, img_, CV_BGR2BGRA);
                cv::resize(img_, img_, cv::Size(img_.cols, img_.rows));
                marker_img.header.frame_id = "/curFrame";
                marker_img.num = 1079.5 * (3.0 / 4.0) * ((float)img_w / img_ori_w);
                marker_img.header.stamp = ros::Time::now();
                sensor_msgs::Image rviz_img;
                rviz_img.height = img_.rows;
                rviz_img.width = img_.cols;
                rviz_img.encoding = "8UC4";
                unsigned char *p = img_.ptr<unsigned char>(0);
                std::vector<unsigned char> vec(p, p + img_.cols * 4 * img_.rows);
                rviz_img.data = vec;
                rviz_img.step = img_.cols * 4;
                marker_img.image = rviz_img;
                std::vector<Eigen::Vector3f> mps = map_upd_match_mp[current_frameId_];
                marker_img.points.reserve(mps.size());
                for (int i = 0; i < mps.size(); i++)
                {
                    geometry_msgs::Point p2;
                    p2.x = mps[i].x();
                    p2.y = mps[i].y();
                    p2.z = mps[i].z();
                    marker_img.points.push_back(p2);
                }
                marker_pub.publish(marker_img);
            }
            else
            {
                //std::cout << "cannot find match from img, error" << std::endl;
            }
        }
        else
        {
        }
    }
    else
    {
    }
}

void localization_server::on_update(const loc_server::loc_update_msg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    if (first_img_flag == 0)
    {
        start_frameId = msg->frame_id;
        first_img_flag = 1;
        std::cout << "start_frameId(from on_update)=" << start_frameId << std::endl;
        if (first_time_flag == 0)
        {
            start_time = ros::Time::now();
            first_time_flag = 1;
            std::cout << "start_time(from on_update)." << std::endl;
        }
    }
    current_frameId = msg->frame_id;
    int current_frameId_ = current_frameId;

    static int last_framId = -1;
    if (last_framId > 0 && last_framId > current_frameId_)
    {
        clearBuffer(0);
        //need_clear_map = true;
        std_msgs::String start_msg;
        start_msg.data = std::string("new_run");
        start_newRun_pub.publish(start_msg);
        std::cout << "clear old data upon new run!" << std::endl;
    }
    last_framId = current_frameId_;

    geometry_msgs::Point update_posi = msg->pose.position;
    map_upd_posi[msg->frame_id] = Eigen::Vector3f(update_posi.x, update_posi.y, update_posi.z);

    geometry_msgs::Quaternion update_dir = msg->pose.orientation;
    map_upd_dir[msg->frame_id].x() = update_dir.x;
    map_upd_dir[msg->frame_id].y() = update_dir.y;
    map_upd_dir[msg->frame_id].z() = update_dir.z;
    map_upd_dir[msg->frame_id].w() = update_dir.w;

    geometry_msgs::Point32 update_cov = msg->cov;
    map_upd_cov[msg->frame_id] = Eigen::Vector3f(update_cov.x, update_cov.y, update_cov.z);

    geometry_msgs::Point32 update_posi_vis = msg->posi_vis;
    map_upd_posi_vis[msg->frame_id] = Eigen::Vector3f(update_posi_vis.x, update_posi_vis.y, update_posi_vis.z);

    upd_cov_vis_map[msg->frame_id] = msg->cov_vis;

    map_inlier_vec[msg->frame_id] = msg->vecInlierIdx;
    /*if((ros::Time::now() - cur_time).toSec() > 5) {
        cur_time = ros::Time::now();
        std::cout << "------>inlier count of frame " << msg->frame_id << " = " << msg->vecInlierIdx.size() << std::endl;
        std::cout << "------>keypoint of frame " << msg->frame_id << " = " << msg->us.size() << std::endl;
    }*/

    std::vector<cv::KeyPoint> upd_keypoint_img;
    for (int i = 0; i < msg->us.size(); i++)
    {
        cv::KeyPoint upd_keypoint_img_;
        upd_keypoint_img_.pt.x = msg->us[i] * img_w / img_ori_w;
        upd_keypoint_img_.pt.y = msg->vs[i] * img_h / img_ori_h;
        upd_keypoint_img.push_back(upd_keypoint_img_);
    }
    map_upd_match_kp[msg->frame_id] = upd_keypoint_img;
    //     std::cout<<"upd_keypoint_img.size::"<<upd_keypoint_img.size()<<std::endl;
    std::vector<Eigen::Vector3f> upd_mappoint_img;
    for (int i = 0; i < msg->us.size(); i++)
    {
        Eigen::Vector3f upd_mappoint_img_ = Eigen::Vector3f(msg->mps[i].x, msg->mps[i].y, msg->mps[i].z);
        upd_mappoint_img.push_back(upd_mappoint_img_);
    }
    map_upd_match_mp[msg->frame_id] = upd_mappoint_img;
    on_display(current_frameId_);

    //     sensor_msgs::PointCloud traj_upd_msg;
    //     traj_upd_msg.header.frame_id = "/world";
    //     traj_upd_msg.points.resize(map_upd_posi.size());
    //     int traj_node_count=0;
    //     for(auto posi_it=map_upd_posi.begin(); posi_it!=map_upd_posi.end();posi_it++){
    //         traj_upd_msg.points[traj_node_count].x = posi_it->second.x();
    //         traj_upd_msg.points[traj_node_count].y = posi_it->second.y();
    //         traj_upd_msg.points[traj_node_count].z = posi_it->second.z();
    //         traj_node_count++;
    //     }
    //     update_traj_pub.publish(traj_upd_msg);

    visualization_msgs::Marker update_traj_msg;
    update_traj_msg.header.frame_id = "/world";
    update_traj_msg.header.stamp = ros::Time::now();
    update_traj_msg.ns = "trajectory";
    update_traj_msg.id = 0;
    update_traj_msg.type = visualization_msgs::Marker::LINE_STRIP;
    update_traj_msg.action = visualization_msgs::Marker::ADD;
    update_traj_msg.pose.position.x = 0;
    update_traj_msg.pose.position.y = 0;
    update_traj_msg.pose.position.z = 0;
    update_traj_msg.pose.orientation.x = 0;
    update_traj_msg.pose.orientation.y = 0;
    update_traj_msg.pose.orientation.z = 0;
    update_traj_msg.pose.orientation.w = 1;
    update_traj_msg.scale.x = 0.3;
    update_traj_msg.scale.y = 0.3;
    update_traj_msg.scale.z = 0.3;
    update_traj_msg.color.r = 1;
    update_traj_msg.color.g = 0;
    update_traj_msg.color.b = 0;
    update_traj_msg.color.a = 1;
    update_traj_msg.lifetime = ros::Duration();
    update_traj_msg.header.stamp = ros::Time::now();
    update_traj_msg.points.resize(map_upd_posi.size());
    int traj_node_count = 0;
    for (auto posi_it = map_upd_posi.begin(); posi_it != map_upd_posi.end(); posi_it++)
    {
        update_traj_msg.points[traj_node_count].x = posi_it->second.x();
        update_traj_msg.points[traj_node_count].y = posi_it->second.y();
        update_traj_msg.points[traj_node_count].z = posi_it->second.z();
        traj_node_count++;
    }
    update_traj_pub.publish(update_traj_msg);

    int cov_count = 0;
    vis_states.markers.clear();
    for (auto cov_it = upd_cov_vis_map.rbegin(); cov_it != upd_cov_vis_map.rend(); cov_it++)
    {
        if (cov_count > 100)
        {
            break;
        }
        float cov = cov_it->second;

        visualization_msgs::Marker vis_state;
        vis_state.header.frame_id = "/world";
        vis_state.header.stamp = ros::Time::now();
        vis_state.ns = "vis_cov";
        vis_state.id = cov_count;
        vis_state.type = visualization_msgs::Marker::SPHERE;
        vis_state.action = visualization_msgs::Marker::ADD;
        auto it_vis_posi = map_upd_posi_vis.find(cov_it->first);
        if (it_vis_posi == map_upd_posi_vis.end())
        {
            continue;
        }
        Eigen::Vector3f his_posi = it_vis_posi->second;
        vis_state.pose.position.x = his_posi.x();
        vis_state.pose.position.y = his_posi.y();
        vis_state.pose.position.z = his_posi.z();
        vis_state.pose.orientation.x = 0;
        vis_state.pose.orientation.y = 0;
        vis_state.pose.orientation.z = 0;
        vis_state.pose.orientation.w = 1;
        float mid_cov_scale = cov * 3;
        if (mid_cov_scale > 10)
        {
            mid_cov_scale = 10;
        }
        vis_state.scale.x = mid_cov_scale;
        vis_state.scale.y = mid_cov_scale;
        vis_state.scale.z = mid_cov_scale;
        vis_state.color.r = 1.0f;
        vis_state.color.g = 0.0f;
        vis_state.color.b = 0.0f;
        vis_state.color.a = 0.5;
        vis_state.lifetime = ros::Duration();
        vis_state.header.stamp = ros::Time::now();
        vis_states.markers.push_back(vis_state);
        cov_count++;
    }
    marker_vis_pub.publish(vis_states);

    visualization_msgs::Marker lines_msg;
    lines_msg.header.frame_id = "/world";
    lines_msg.header.stamp = ros::Time::now();
    lines_msg.ns = "roadPaint";
    lines_msg.id = 1;
    lines_msg.type = visualization_msgs::Marker::LINE_LIST;
    lines_msg.action = visualization_msgs::Marker::ADD;
    lines_msg.pose.position.x = 0;
    lines_msg.pose.position.y = 0;
    lines_msg.pose.position.z = 0;
    lines_msg.pose.orientation.x = 0;
    lines_msg.pose.orientation.y = 0;
    lines_msg.pose.orientation.z = 0;
    lines_msg.pose.orientation.w = 1;
    lines_msg.scale.x = 0.05;
    lines_msg.scale.y = 0.05;
    lines_msg.scale.z = 0.05;
    lines_msg.color.r = 1.0f;
    lines_msg.color.g = 0.0f;
    lines_msg.color.b = 0.0f;
    lines_msg.color.a = 1;
    lines_msg.lifetime = ros::Duration();
    lines_msg.header.stamp = ros::Time::now();
    int posi_count = 0;
    for (auto posi_it = map_upd_posi_vis.rbegin(); posi_it != map_upd_posi_vis.rend(); posi_it++)
    {
        if (posi_count > 100)
        {
            break;
        }
        geometry_msgs::Point p2;
        p2.x = posi_it->second.x();
        p2.y = posi_it->second.y();
        p2.z = posi_it->second.z();
        auto it_upd_posi = map_upd_posi.find(posi_it->first);
        if (it_upd_posi == map_upd_posi.end())
        {
            continue;
        }
        Eigen::Vector3f his_posi = it_upd_posi->second;
        geometry_msgs::Point p3;
        p3.x = his_posi.x();
        p3.y = his_posi.y();
        p3.z = his_posi.z();
        lines_msg.points.push_back(p2);
        lines_msg.points.push_back(p3);
        posi_count++;
    }
    marker_line_pub.publish(lines_msg);
}

void localization_server::on_gps(const geometry_msgs::Point32::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    time_receive_gps = ros::Time::now();
    //     std::cout<<"time_receive_gps:"<<time_receive_gps<<std::endl;
    //     std::cout<<"time_receive_predict:"<<time_receive_predict<<std::endl;
    if ((time_receive_gps - time_receive_predict).toSec() > 3)
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "curFrame";
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = msg->x;
        transformStamped.transform.translation.y = msg->y;
        transformStamped.transform.translation.z = -msg->z;
        // transformStamped.transform.rotation.x = 0;
        // transformStamped.transform.rotation.y = 0;
        // transformStamped.transform.rotation.z = 0;
        // transformStamped.transform.rotation.w = 1;

        transformStamped.transform.rotation.x = -0.0154253;
        transformStamped.transform.rotation.y = -0.706932;
        transformStamped.transform.rotation.z = 0.706932;
        transformStamped.transform.rotation.w = -0.0154253;

        curPose.sendTransform(transformStamped);
    }
    vec_gps_posi.push_back(Eigen::Vector3f(msg->x, msg->y, -msg->z));
    if (first_time_flag == 0)
    {
        start_time = ros::Time::now();
        first_time_flag = 1;
        std::cout << "start_time(from on_gps)." << std::endl;
    }
    sensor_msgs::PointCloud gps;
    gps.header.frame_id = "/world";
    gps.points.resize(vec_gps_posi.size());
    for (int i = 0; i < vec_gps_posi.size(); i++)
    {
        gps.points[i].x = vec_gps_posi[i].x();
        gps.points[i].y = vec_gps_posi[i].y();
        gps.points[i].z = vec_gps_posi[i].z();
    }

    gps_pub.publish(gps);
}

void localization_server::on_rviz_control(const loc_server::rviz_control_msg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    if (msg->Loc_match_switch == 1)
        Loc_match_switch = 1;
    if (msg->Loc_match_switch == 2)
        Loc_match_switch = 0;
    if (msg->Upd_match_switch == 1)
        Upd_match_switch = 1;
    if (msg->Upd_match_switch == 2)
        Upd_match_switch = 0;
    if (msg->Iamge_id_switch == 1)
        Image_id_switch = 1;
    if (msg->Iamge_id_switch == 2)
        Image_id_switch = 0;
    on_display(current_frameId);
}
void localization_server::on_image(const loc_server::server_img_msg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    if (first_img_flag == 0)
    {
        start_frameId = msg->frame_id;
        first_img_flag = 1;
        std::cout << "start_frameId(from on_image)=" << start_frameId << std::endl;
        if (first_time_flag == 0)
        {
            start_time = ros::Time::now();
            first_time_flag = 1;
            std::cout << "start_time(from on_image)." << std::endl;
        }
    }
    current_frameId = msg->frame_id;
    //     std::cout<<"current_frameId:"<<current_frameId<<std::endl;
    int current_frameId_ = current_frameId;
    if (read_img_from_ros)
    {
        /*        cv::Mat img_from_ros;        
        std::string img_name = image_input_path + std::to_string(current_frameId)+".jpg";
        std::cout<<"img_name:"<<img_name<<std::endl;
        img_from_ros = cv::imread(img_name);
        if(img_from_ros.empty())
            return;        
        cv::resize(img_from_ros, img_from_ros, cv::Size(img_w,img_h));
        map_img[msg->frame_id] = img_from_ros;*/
    }
    else
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg->image, "");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        scale_down = msg->scale;
        cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(img_w, img_h));
        //cv::waitKey(1);
        map_img[msg->frame_id] = cv_ptr->image;
    }
    cv::waitKey(1);
    on_display(current_frameId_);
}

void localization_server::on_matches(const loc_server::match_info_msg::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    if (read_img_from_ros == false)
    {
        if (first_img_flag == 0)
        {
            start_frameId = msg->frame_id;
            first_img_flag = 1;
            std::cout << "start_frameId(from on_matches)=" << start_frameId << std::endl;
            if (first_time_flag == 0)
            {
                start_time = ros::Time::now();
                first_time_flag = 1;
                std::cout << "start_time(from on_matches)." << std::endl;
            }
        }
        if (msg->match_type == 1)
        {
            current_frameId = msg->frame_id;
            int current_frameId_ = current_frameId;
            std::vector<cv::KeyPoint> loc_keypoint_img;
            for (int i = 0; i < msg->us.size(); i++)
            {
                cv::KeyPoint loc_keypoint_img_;
                loc_keypoint_img_.pt.x = msg->us[i] * img_w / img_ori_w;
                loc_keypoint_img_.pt.y = msg->vs[i] * img_h / img_ori_h;
                loc_keypoint_img.push_back(loc_keypoint_img_);
            }
            map_loc_match_kp[msg->frame_id] = loc_keypoint_img;

            std::vector<Eigen::Vector3f> loc_mappoint_img;
            for (int i = 0; i < msg->us.size(); i++)
            {
                Eigen::Vector3f loc_mappoint_img_ = Eigen::Vector3f(msg->mps[i].x, msg->mps[i].y, msg->mps[i].z);
                loc_mappoint_img.push_back(loc_mappoint_img_);
            }
            map_loc_match_mp[msg->frame_id] = loc_mappoint_img;
            on_display(current_frameId_);
        }
    }
}

void localization_server::on_kfposition(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    if (msg->points.size() == 0)
    {
        std::cout<<"set kf clear flag"<<std::endl;
        need_clear_kf = true;
        return;
    }
    if (need_clear_kf)
    {
        std::cout<<"clear kf"<<std::endl;
        kf_mp.points.clear();
        kf_mp.channels[0].values.clear();
        need_clear_kf = false;
    }

    kf_mp.header.frame_id = "world";
    kf_mp.points.insert(std::end(kf_mp.points), std::begin((*msg).points), std::end((*msg).points));
    if((*msg).channels.size()>0){
        kf_mp.channels[0].values.insert(std::end(kf_mp.channels[0].values), std::begin((*msg).channels[0].values), std::end((*msg).channels[0].values));
    }
    std::cout<<"get db kfs: "<<kf_mp.points.size()<<std::endl;
}

void localization_server::on_database(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_i_mutex);

    if (msg->points.size() == 0)
    {
        // need_clear_map = true;
        return;
    }
    if (need_clear_map)
    {
        database_mp.points.clear();
        database_mp.channels[0].values.clear();
        std::cout << "clear old database" << std::endl;
        need_clear_map = false;
    }
    std::cout << "get database" << std::endl;
    database_mp.header.frame_id = "world";
    database_mp.points.insert(std::end(database_mp.points), std::begin((*msg).points), std::end((*msg).points));
    if((*msg).channels.size()>0){
        database_mp.channels[0].values.insert(std::end(database_mp.channels[0].values), std::begin((*msg).channels[0].values), std::end((*msg).channels[0].values));
    }
    std::cout << "mp count: " << database_mp.points.size() << std::endl;
    first_img_flag = 0;
    start_time = ros::Time::now();
    first_time_flag = 1;

    //clearBuffer(0);
}

localization_server::localization_server()
{
    vec_gps_posi.resize(max_frame_size);
    imu_count = 0;
    gps_count = 0;
    ros::NodeHandle n;
    imu_traj_pub = n.advertise<sensor_msgs::PointCloud>("imu_traj", 1);
    //update_traj_pub = n.advertise<sensor_msgs::PointCloud>("update_traj", 1);
    update_traj_pub = n.advertise<visualization_msgs::Marker>("update_traj", 1);
    gps_pub = n.advertise<sensor_msgs::PointCloud>("Gps", 1);
    marker_pub = n.advertise<veh_msg::veh_msg>("imu_marker", 1);
    mps_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    img_pub = n.advertise<sensor_msgs::Image>("cur/image", 1);
    marker_line_pub = n.advertise<visualization_msgs::Marker>("ekf_lines", 1);
    marker_imu_pub = n.advertise<visualization_msgs::MarkerArray>("imu_marker1", 1);
    current_uncertainty = n.advertise<visualization_msgs::Marker>("current_uncertainty", 1);
    marker_vis_pub = n.advertise<visualization_msgs::MarkerArray>("vis_marker1", 1);
    posi_show_mp_pub = n.advertise<geometry_msgs::Point>("posi_show_map", 1);
    kfposition_pub = n.advertise<sensor_msgs::PointCloud>("kfposition", 1);
    imu_indicator_pub = n.advertise<std_msgs::String>("imu_indicator", 1);
    gps_indicator_pub = n.advertise<std_msgs::String>("gps_indicator", 1);
    start_newRun_pub = n.advertise<std_msgs::String>("new_run", 1);

    sub_predict = n.subscribe<loc_server::loc_predict_msg>("predict_server", 100, &localization_server::on_predict, this);
    sub_update = n.subscribe<loc_server::loc_update_msg>("update_server", 100, &localization_server::on_update, this);
    sub_match = n.subscribe<loc_server::match_info_msg>("match_server", 100, &localization_server::on_matches, this);
    sub_gps = n.subscribe<geometry_msgs::Point32>("gps_server", 100, &localization_server::on_gps, this);
    sub_image = n.subscribe<loc_server::server_img_msg>("image_server", 100, &localization_server::on_image, this);
    sub_rviz_control = n.subscribe<loc_server::rviz_control_msg>("rviz_control", 100, &localization_server::on_rviz_control, this);
    sub_database = n.subscribe<sensor_msgs::PointCloud>("database_server", 100, &localization_server::on_database, this);
    sub_kfposition = n.subscribe<sensor_msgs::PointCloud>("kfposition_server", 100, &localization_server::on_kfposition, this);
    sub_packer = n.subscribe<record_msg::data_record>("data_record", 100, &localization_server::on_packer, this);

    need_clear_map = false;
    need_clear_kf = false;
    kf_mp.channels.resize(1);
    kf_mp.channels[0].name = "rgb";
    database_mp.channels.resize(1);
    database_mp.channels[0].name = "rgb";
    cur_time = ros::Time::now();


    read_img_from_ros = LocContext::getInstance()->getParam("Control", "switch_run_Asample") == "True";
    resize_scale = std::stof(LocContext::getInstance()->getParam("Control", "scale_resize"));
    cx = 6.620275635286945e+02 * resize_scale;
    cy = 3.911259739288044e+02 * resize_scale;
    fx = 1.102885481033972e+03 * resize_scale;
    fy = 1.103520623055449e+03 * resize_scale;
    img_ori_w = 1280 * resize_scale;
    img_ori_h = 800 * resize_scale;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loc_server");
    ros::NodeHandle n;
    ros::Rate r(60);
    localization_server my_server;
    std::cout << "start loop!!" << std::endl;
    static int del_time_cont = 0;
    my_server.first_img_flag = 0;
    my_server.first_time_flag = 0;

    while (ros::ok())
    {
        del_time_cont++;
        if (del_time_cont > 120)
        {
            del_time_cont = 0;
            if (my_server.database_mp.points.size() > 0)
            {
                my_server.mps_pub.publish(my_server.database_mp);
            }
            if (my_server.kf_mp.points.size() > 0)
            {
                my_server.kfposition_pub.publish(my_server.kf_mp);
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    std::cout << "end loop!!" << std::endl;
    return 0;
};
