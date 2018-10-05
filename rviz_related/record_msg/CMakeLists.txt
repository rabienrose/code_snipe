cmake_minimum_required(VERSION 2.8.3)
project(record_msg)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs)
add_message_files(DIRECTORY msg
FILES
imu_record.msg
img_record.msg
gps_record.msg
imu_pack.msg
data_record.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

