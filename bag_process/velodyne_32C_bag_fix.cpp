#include <iostream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <rosgraph_msgs/Log.h>


int main(int argc, char **argv)
{
    if (2 != argc) {
        std::cout << "Usage:" << std::endl;
        std::cout << "    ./velodyne_32C_bag_fix + <*.bag>" << std::endl;
    }

    std::clock_t time = std::clock();

    std::string in_bag_name {argv[1]};
    std::string out_bag_name {argv[1]};
    size_t pos = out_bag_name.find(".bag");
    if (std::string::npos == pos) {
        std::cout << "The bag file name is not end with '*.bag' " << std::endl;
        return 1;
    }
    out_bag_name.insert(pos, "_fixed");

    std::cout << "Input bag:  " << in_bag_name << std::endl;
    std::cout << "Output bag: " << out_bag_name << std::endl;

    rosbag::Bag in_bag {in_bag_name, rosbag::bagmode::Read};
    rosbag::Bag out_bag {out_bag_name, rosbag::bagmode::Write};

    std::cout << "Starting fixing" << std::endl;

    for (rosbag::MessageInstance m : rosbag::View{in_bag}) {
        if (m.getTopic() == "/imu/data" || m.getTopic() == "imu/data" ||
            m.getTopic() == "/imu/raw_data" || m.getTopic() == "imu/raw_data") {

            out_bag.write(m.getTopic(), m.getTime(), m.instantiate<sensor_msgs::Imu>(),
                          m.getConnectionHeader());
            continue;
        }

        if (m.getTopic() == "/camera/left/image_raw" || m.getTopic() == "camera/left/image_raw" ||
            m.getTopic() == "/camera/right/image_raw" || m.getTopic() == "camera/right/image_raw") {

            out_bag.write(m.getTopic(), m.getTime(), m.instantiate<sensor_msgs::Image>(),
                          m.getConnectionHeader());
            continue;
        }

        if (m.getTopic() == "/rosout" || m.getTopic() == "rosout" ) {
            out_bag.write(m.getTopic(), m.getTime(), m.instantiate<rosgraph_msgs::Log>(),
                          m.getConnectionHeader());
            continue;
        }
        if (m.getTopic() == "/velodyne_points" || m.getTopic() == "velodyne_points" ) {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();

            for (uint32_t height = 0; height < msg->height; ++height) {
                uint8_t* data_ptr = &msg->data[height * msg->row_step];
                for (uint32_t width = 0; width < msg->width; ++width) {
                    uint8_t* point_ptr = data_ptr + width * msg->point_step;

                    for (const auto& field : msg->fields) {
                        if ("x" == field.name || "y" == field.name || "z" == field.name) {
                            float* xyz = reinterpret_cast<float*>(point_ptr + field.offset);
                            *xyz *= 2.;
                        }
                    }
                }
            }
            out_bag.write(m.getTopic(), m.getTime(), msg, m.getConnectionHeader());
        }
    }
    in_bag.close();
    out_bag.close();

    std::cout << "Process time: " << (std::clock() - time) / 1000000 << "s" << std::endl;
    std::cout << "Done" << std::endl;
};
