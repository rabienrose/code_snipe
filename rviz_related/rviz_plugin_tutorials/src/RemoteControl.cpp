#include <stdio.h>
#include <qt4/QtGui/QCheckBox>

#include <QPainter>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QDialog>
#include <QTimer>
#include <QScrollBar>
#include <std_msgs/String.h>
#include <stdio.h>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include "RemoteControl.h"
#include "remoteConHelper.h"
#include "context.h"

namespace rviz_plugin_tutorials
{

void RemoteControl::resetStatus()
{
    delay_result->setText("0/0");
    status_result->setText("No Data");
    //gps_result->setText("0");
    //imu_result->setText("0");
    //Debug_result->setText("");
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (getline(ss, item, delim))
    {
        tokens.push_back(item);
    }
    return tokens;
}

void RemoteControl::Button_temp_event()
{
}

void RemoteControl::Button_reboot_event()
{
    rebootMac();
}

RemoteControl::RemoteControl(QWidget *parent)
    : rviz::Panel(parent)
{
    QPushButton *Button_recv = new QPushButton;
    Button_recv->setText("Recv");
    connect(Button_recv, SIGNAL(clicked()), this, SLOT(Button_ros_data_sender_event()));

    QPushButton *Button_record = new QPushButton;
    Button_record->setText("Record");
    connect(Button_record, SIGNAL(clicked()), this, SLOT(Button_rosbag_event()));

    QPushButton *Button_loc = new QPushButton;
    Button_loc->setText("Loc");
    connect(Button_loc, SIGNAL(clicked()), this, SLOT(Button_ros_loc_algo_event()));

    QPushButton *Button_recv_loc = new QPushButton;
    Button_recv_loc->setText("Recv & Loc");
    connect(Button_recv_loc, SIGNAL(clicked()), this, SLOT(Button_ros_real_loc_event()));

    QLabel *database_list_cbo_info = new QLabel(QWidget::tr("database_list:"));
    database_list_cbo = new QComboBox();
    connect(database_list_cbo, SIGNAL(currentIndexChanged(int)), this, SLOT(database_list_cbo_valueChanged()));

    QLabel *delay_info = new QLabel(QWidget::tr("Delay:"));
    delay_result = new QLabel;
    delay_result->setText("0/0");

    QLabel *status_info = new QLabel(QWidget::tr("Status:"));
    status_result = new QLabel;
    status_result->setText("No Data");

    QLabel *gps_info = new QLabel(QWidget::tr("GPS:"));
    gps_result = new QLabel;
    gps_result->setText("0");

    QLabel *imu_info = new QLabel(QWidget::tr("IMU:"));
    imu_result = new QLabel;
    imu_result->setText("0");

    QPushButton *Button_reboot = new QPushButton;
    Button_reboot->setText("Reboot MAC");
    connect(Button_reboot, SIGNAL(clicked()), this, SLOT(Button_reboot_event()));

    QPushButton *Button_temp = new QPushButton;
    Button_temp->setText("Temp");
    connect(Button_temp, SIGNAL(clicked()), this, SLOT(Button_temp_event()));

    Debug_result = new QTextEdit;

    QGridLayout *gridLayout = new QGridLayout;
    gridLayout->setColumnStretch(0, 1); //表示第0列和第1列的比例
    gridLayout->setColumnStretch(1, 1);

    gridLayout->setMargin(10);  //表示控件与窗体的左右边距
    gridLayout->setSpacing(10); //设置组件的间隔为10像素

    gridLayout->addWidget(Button_record);
    gridLayout->addWidget(Button_recv);
    gridLayout->addWidget(Button_loc);
    gridLayout->addWidget(Button_recv_loc);
    gridLayout->addWidget(database_list_cbo_info);   //
    gridLayout->addWidget(database_list_cbo);        //
    gridLayout->addWidget(delay_info);               //
    gridLayout->addWidget(delay_result);             //
    gridLayout->addWidget(status_info);              //
    gridLayout->addWidget(status_result);            //
    gridLayout->addWidget(gps_info);                 //
    gridLayout->addWidget(gps_result);               //
    gridLayout->addWidget(imu_info);                 //
    gridLayout->addWidget(imu_result);               //
    gridLayout->addWidget(Button_reboot);            //
    gridLayout->addWidget(Button_temp);              //
    gridLayout->addWidget(Debug_result, 8, 0, 1, 2); //

    setLayout(gridLayout);

    remoteConHelper *con = remoteConHelper::getInst();
    for (int i = 0; i < con->database_list.size(); i++)
    {
        if (i == 0)
        {
            cur_database = con->database_list[i].c_str();
        }
        database_list_cbo->addItem(QWidget::tr(con->database_list[i].c_str()));
    }
}

void RemoteControl::on_imu_status(const std_msgs::String::ConstPtr &msg)
{
    imu_result->setText(msg->data.c_str());
}

void RemoteControl::on_gps_status(const std_msgs::String::ConstPtr &msg)
{
    gps_result->setText(msg->data.c_str());
}

void RemoteControl::on_add_error(const std_msgs::String::ConstPtr &msg)
{
    showMessage((msg->data + "\n").c_str());
}

void RemoteControl::on_add_delay(const std_msgs::String::ConstPtr &msg)
{
    delay_result->setText(msg->data.c_str());
}

void RemoteControl::on_add_status(const std_msgs::String::ConstPtr &msg)
{
    status_result->setText(msg->data.c_str());
}

void RemoteControl::Button_ros_data_sender_event()
{
    if (remoteConHelper::getInst()->tryKillNode("/ros_data_sender"))
    {
        showMessage("Stop recv\n");
        return;
    }
    std::stringstream ss;
    ss << remoteConHelper::getInst()->set_ros_ip_str << " && " << LocContext::getInstance()->getParam("Control", "code_path") << "examples/RosDataSender/RosDataSender";
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
    resetStatus();
    Debug_result->setText("");
    showMessage("Start recv\n");
}
void RemoteControl::Button_rosbag_event()
{
    std::vector<std::string> re_lines = remoteConHelper::getInst()->do_cmd("rosnode list");
    bool kill_some_node = false;
    for (int i = 0; i < re_lines.size(); i++)
    {
        if (re_lines[i][1] == 'r' && re_lines[i][2] == 'e' && re_lines[i][3] == 'c' && re_lines[i][4] == 'o' && re_lines[i][5] == 'r' && re_lines[i][6] == 'd' && re_lines[i][7] == '_')
        {
            std::stringstream ss;
            ss << "rosnode kill " << re_lines[i];
            re_lines = remoteConHelper::getInst()->do_cmd(ss.str());
            kill_some_node = true;
        }
        if (kill_some_node == true)
        {
            showMessage("Stop recording\n");
            return;
        }
    }

    std::string record = "/Users/test/Documents/work/ros_package/ros/install_isolated/lib/rosbag/record --buffsize 256 --chunksize 768 /data_record";
    std::stringstream ss;
    ss << remoteConHelper::getInst()->set_ros_ip_str << " && " << record;
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
    resetStatus();
    showMessage("Start recording\n");
}

void RemoteControl::showMessage(std::string text)
{
    Debug_result->insertPlainText(text.c_str());
    Debug_result->verticalScrollBar()->setValue(Debug_result->verticalScrollBar()->maximum());
}

void RemoteControl::Button_ros_loc_algo_event()
{
    if(remoteConHelper::getInst()->tryKillNode("/loc_algo_ros")){
        showMessage("Stop loc\n");
        return;
    }

    std::string code_path = LocContext::getInstance()->getParam("Control", "code_path");
    std::string database_path = LocContext::getInstance()->getParam("Control", "database_path");
    std::string bow_file = LocContext::getInstance()->getParam("Control", "bow_file");

    std::stringstream ss;
    ss<<remoteConHelper::getInst()->set_ros_ip_str<<" && "<<code_path<<"LocRealTimeRosAlgo"<<" "<<
        code_path<<"bin/resources/CONTI/CONTI65.yaml"<<" "<<
        code_path<<"config/slamConfig.json"<<" "<<
        database_path<<cur_database<<" "<<
        "none"<<" "<<
        "none"<<" "<<
        "0"<<" "<<
        "100000"<<" "<<
        bow_file<<" "<<
        code_path<<"config/locConfigRosViewer.json"<<" "<<
        code_path<<"config/IMUConfig_Conti65.json"<<" "<<
        "none";
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
    resetStatus();
    std::stringstream ss1;
    ss1 << "Start loc. Database: " << cur_database << std::endl;
    showMessage(ss1.str().c_str());
}
void RemoteControl::Button_ros_real_loc_event()
{
    if (remoteConHelper::getInst()->tryKillNode("/loc_realtime"))
    {
        showMessage("Stop running\n");
        return;
    }

    std::string code_path = LocContext::getInstance()->getParam("Control", "code_path");
    std::string database_path = LocContext::getInstance()->getParam("Control", "database_path");
    std::string bow_file = LocContext::getInstance()->getParam("Control", "bow_file");

    std::stringstream ss;
    ss << remoteConHelper::getInst()->set_ros_ip_str << " && " << code_path << "examples/LocRealTime/loc_realtime"
       << " " << code_path << "bin/resources/CONTI/CONTI65.yaml"
       << " " << code_path << "config/slamConfig.json"
       << " " << database_path << cur_database << " "
       << "none"
       << " "
       << "none"
       << " "
       << "0"
       << " "
       << "100000"
       << " " << bow_file << " " << code_path << "config/locConfigRosViewer.json"
       << " " << code_path << "config/IMUConfig_Conti65.json"
       << " "
       << "none";
    remoteConHelper::getInst()->do_cmd(ss.str(), true, false);
    resetStatus();
    Debug_result->setText("");
    std::stringstream ss1;
    ss1 << "Start running. Database: " << cur_database << std::endl;
    showMessage(ss1.str().c_str());
}
void RemoteControl::database_list_cbo_valueChanged()
{
    std_msgs::String msg;
    QString cur_database_name = database_list_cbo->currentText();
    cur_database = cur_database_name.toUtf8().constData();
}

void RemoteControl::rebootMac()
{
    remoteConHelper::getInst()->do_cmd("sudo reboot", true);
}

void RemoteControl::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void RemoteControl::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    error_msg_sub = nh_.subscribe<std_msgs::String>("add_error", 100, &RemoteControl::on_add_error, this);
    log_status_sub = nh_.subscribe<std_msgs::String>("add_status", 100, &RemoteControl::on_add_status, this);
    dalay_sub = nh_.subscribe<std_msgs::String>("add_delay", 100, &RemoteControl::on_add_delay, this);
    imu_sub = nh_.subscribe<std_msgs::String>("imu_indicator", 100, &RemoteControl::on_imu_status, this);
    gps_sub = nh_.subscribe<std_msgs::String>("gps_indicator", 100, &RemoteControl::on_gps_status, this);
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::RemoteControl, rviz::Panel)
// END_TUTORIAL
